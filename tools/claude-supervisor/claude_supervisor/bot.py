"""Discord bot: notifications, interactive buttons, and authorized text commands.

Single consolidated bot -- it both sends alerts and listens for commands. Every
command and every button click is gated on the single authorized user ID; any
other user is silently ignored (and logged).
"""

from __future__ import annotations

import io
import logging

import discord
from discord.ext import commands

from .config import Config
from .core import Supervisor
from .state import SessionMetadata, SessionState

log = logging.getLogger("supervisor.bot")

_REASON_LABEL = {
    "permission": "Permission",
    "clarification": "Clarification",
    "milestone": "Milestone Checkpoint",
    "review": "Review",
    "risk": "Risk Warning",
    "failure": "Failure / Error",
}


def _fmt_elapsed(seconds: float) -> str:
    m, s = divmod(int(seconds), 60)
    if m == 0:
        return f"{s}s"
    return f"{m}m {s}s"


def _clip(text: str, n: int = 1000) -> str:
    text = text or ""
    return text if len(text) <= n else text[: n - 1] + "…"


class SupervisorBot(commands.Bot):
    def __init__(self, config: Config, supervisor: Supervisor) -> None:
        intents = discord.Intents.default()
        intents.message_content = True
        super().__init__(command_prefix="!", intents=intents, help_command=None)
        self.cfg = config
        self.sup = supervisor
        self.authorized_id = int(config.discord.authorized_user_id)
        self._register_commands()

    # ---- auth ------------------------------------------------------------

    def is_authorized(self, user_id: int) -> bool:
        if user_id != self.authorized_id:
            log.warning("Unauthorized access attempt by %s", user_id)
            return False
        return True

    async def setup_hook(self) -> None:
        self.add_view(ActionButtons(self))  # persistent view

    # ---- alerts (Supervisor -> Discord) ----------------------------------

    async def send_alert(
        self,
        meta: SessionMetadata,
        snapshot=None,
        *,
        reminder: bool = False,
    ) -> None:
        channel = self.get_channel(self.cfg.discord.channel_id)
        if channel is None:
            channel = await self.fetch_channel(self.cfg.discord.channel_id)
        prefix = "🔁 Reminder — " if reminder else ""
        reason = meta.waiting_reason.value if meta.waiting_reason else "unknown"
        embed = discord.Embed(
            title=f"{prefix}🚨 Claude Checkpoint",
            color=discord.Color.orange() if not reminder else discord.Color.red(),
        )

        project = (snapshot.project if snapshot else meta.project_name) or "?"
        branch = (snapshot.branch if snapshot else meta.git_branch) or "?"
        embed.add_field(name="Project", value=project, inline=True)
        embed.add_field(name="Branch", value=branch, inline=True)
        embed.add_field(name="Machine", value=meta.machine or "?", inline=True)

        if snapshot and snapshot.completed:
            label = "Recent commits" if snapshot.completed_from_commits else "Completed"
            bullets = "\n".join(f"- {c}" for c in snapshot.completed[:8])
            embed.add_field(name=label, value=_clip(bullets), inline=False)

        current = (snapshot.current_task if snapshot else None) or "—"
        embed.add_field(name="Current", value=_clip(current, 500), inline=False)

        embed.add_field(name="Needs", value=_REASON_LABEL.get(reason, reason), inline=True)
        tests = snapshot.tests if snapshot else "unknown"
        embed.add_field(name="Tests", value=tests, inline=True)
        embed.add_field(name="Waiting", value=_fmt_elapsed(meta.waiting_elapsed_seconds()), inline=True)

        question = meta.last_prompt_text or "(no message captured)"
        embed.add_field(name="Question", value=f"```\n{_clip(question, 980)}\n```", inline=False)

        if snapshot and snapshot.changed_files:
            files = "\n".join(snapshot.changed_files[:12])
            embed.add_field(name="Files", value=_clip(files), inline=False)

        # Ping the authorized user in the message *content* (mentions inside an
        # embed never trigger a Discord notification). ID-based mention so it
        # survives username changes; allowed_mentions restricts the ping to that
        # single ID so nothing in the embed text can ever mention anyone else.
        await channel.send(
            content=f"<@{self.authorized_id}>",
            embed=embed,
            view=ActionButtons(self),
            allowed_mentions=discord.AllowedMentions(
                everyone=False, roles=False, users=[discord.Object(id=self.authorized_id)]
            ),
        )

    # ---- text commands ---------------------------------------------------

    def _register_commands(self) -> None:
        bot = self

        @self.check
        async def _global_auth(ctx: commands.Context) -> bool:
            return bot.is_authorized(ctx.author.id)

        @self.command(name="status")
        async def status(ctx: commands.Context):
            m = bot.sup.meta
            paused = " (alerts PAUSED)" if bot.sup.paused else ""
            await ctx.send(
                f"**State:** {m.current_state.value}{paused}\n"
                f"**Project:** {m.project_name} | **Branch:** {m.git_branch} | **Machine:** {m.machine}\n"
                f"**Reason:** {m.waiting_reason.value if m.waiting_reason else '-'}\n"
                f"**Elapsed:** {_fmt_elapsed(m.waiting_elapsed_seconds())} | **PID:** {m.pid}"
            )

        @self.command(name="approve", aliases=["continue"])
        async def approve(ctx: commands.Context):
            bot.sup.continue_()
            await ctx.send("✅ Sent continue/approve to Claude.")

        @self.command(name="deny")
        async def deny(ctx: commands.Context):
            bot.sup.deny()
            await ctx.send("🚫 Sent deny to Claude.")

        @self.command(name="reply")
        async def reply(ctx: commands.Context, *, message: str = ""):
            if not message:
                await ctx.send("Usage: `!reply <text>`")
                return
            bot.sup.inject_text(message)
            await ctx.send("💬 Injected reply into Claude.")

        @self.command(name="logs")
        async def logs(ctx: commands.Context):
            await bot._send_logs(ctx.send)

        @self.command(name="context")
        async def context(ctx: commands.Context):
            snap = await bot.sup.refresh_snapshot()
            if snap is None:
                await ctx.send("No context snapshot available yet.")
                return
            await bot._send_text(ctx.send, snap.to_text(max_len=100_000), "claude_context.txt")

        @self.command(name="pause")
        async def pause(ctx: commands.Context):
            bot.sup.pause()
            await ctx.send("⏸️ Inactivity alerts paused.")

        @self.command(name="resume")
        async def resume(ctx: commands.Context):
            bot.sup.resume()
            await ctx.send("▶️ Inactivity alerts resumed.")

        @self.command(name="stop")
        async def stop(ctx: commands.Context):
            bot.sup.stop_claude()
            await ctx.send("🛑 Sent Ctrl+C to Claude.")

        @bot.event
        async def on_command_error(ctx, error):
            if isinstance(error, commands.CheckFailure):
                return  # unauthorized -> silently ignore
            if isinstance(error, commands.CommandNotFound):
                return
            log.exception("command error: %r", error)

    async def _send_logs(self, sender) -> None:
        text = self.sup.recent_logs(50) or "(no output captured yet)"
        await self._send_text(sender, text, "claude_logs.txt")

    async def _send_text(self, sender, text: str, filename: str) -> None:
        """Send as a code block if it fits Discord's limit, else as a .txt file."""
        text = text or "(empty)"
        limit = self.cfg.behavior.max_discord_msg_len
        if len(text) <= limit:
            await sender(f"```\n{text}\n```")
        else:
            data = io.BytesIO(text.encode("utf-8"))
            await sender(file=discord.File(data, filename=filename))


class ActionButtons(discord.ui.View):
    """Approve / Deny / Logs / Stop buttons attached to each alert."""

    def __init__(self, bot: SupervisorBot) -> None:
        super().__init__(timeout=None)
        self.bot = bot

    async def _guard(self, interaction: discord.Interaction) -> bool:
        if not self.bot.is_authorized(interaction.user.id):
            await interaction.response.send_message("Not authorized.", ephemeral=True)
            return False
        return True

    @discord.ui.button(label="Approve", style=discord.ButtonStyle.success, custom_id="sup:approve")
    async def approve(self, interaction: discord.Interaction, button: discord.ui.Button):
        if not await self._guard(interaction):
            return
        self.bot.sup.approve()
        await interaction.response.send_message("✅ Approved.", ephemeral=True)

    @discord.ui.button(label="Deny", style=discord.ButtonStyle.danger, custom_id="sup:deny")
    async def deny(self, interaction: discord.Interaction, button: discord.ui.Button):
        if not await self._guard(interaction):
            return
        self.bot.sup.deny()
        await interaction.response.send_message("🚫 Denied.", ephemeral=True)

    @discord.ui.button(label="Logs", style=discord.ButtonStyle.secondary, custom_id="sup:logs")
    async def logs(self, interaction: discord.Interaction, button: discord.ui.Button):
        if not await self._guard(interaction):
            return
        await interaction.response.defer(ephemeral=True)
        await self.bot._send_logs(interaction.followup.send)

    @discord.ui.button(label="Stop", style=discord.ButtonStyle.danger, custom_id="sup:stop")
    async def stop(self, interaction: discord.Interaction, button: discord.ui.Button):
        if not await self._guard(interaction):
            return
        self.bot.sup.stop_claude()
        await interaction.response.send_message("🛑 Ctrl+C sent.", ephemeral=True)
