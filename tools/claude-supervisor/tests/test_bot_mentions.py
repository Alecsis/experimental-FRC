"""Regression tests for the Discord alert *mention/ping* behavior.

Verifies that both checkpoint and reminder alerts ping the authorized user via an
ID-based mention placed in the message content (mentions inside an embed do not
notify), that the mention does not disturb the embed, and that allowed_mentions
restricts the ping to only the configured authorized_user_id.

No live Discord connection: SupervisorBot is constructed offline and get_channel
is stubbed to a recorder. Requires discord.py installed.

Run: python tests/test_bot_mentions.py
"""

from __future__ import annotations

import asyncio
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import discord  # noqa: E402

from claude_supervisor.bot import SupervisorBot  # noqa: E402
from claude_supervisor.config import (  # noqa: E402
    BehaviorConfig,
    Config,
    DiscordConfig,
    SupervisorConfig,
    TimersConfig,
)

AUTHORIZED_ID = "878806618714820638"

PASS = 0
FAIL = 0


def check(cond, label):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS: {label}")
    else:
        FAIL += 1
        print(f"  FAIL: {label}")


class RecordingChannel:
    def __init__(self):
        self.sends = []

    async def send(self, content=None, *, embed=None, view=None, allowed_mentions=None):
        self.sends.append(
            {"content": content, "embed": embed, "view": view, "allowed_mentions": allowed_mentions}
        )


def _make_cfg():
    return Config(
        discord=DiscordConfig(bot_token="tok", authorized_user_id=AUTHORIZED_ID, channel_id=999),
        timers=TimersConfig(),
        behavior=BehaviorConfig(),
        supervisor=SupervisorConfig(),
        ipc_token="",
    )


def _fake_meta():
    meta = types.SimpleNamespace(
        waiting_reason=types.SimpleNamespace(value="milestone"),
        project_name="proj",
        git_branch="main",
        machine="box",
        last_prompt_text="Analyze this repo. Ask before continuing.",
    )
    meta.waiting_elapsed_seconds = lambda: 12.0
    return meta


async def _send_and_capture(reminder: bool):
    bot = SupervisorBot(_make_cfg(), supervisor=object())
    channel = RecordingChannel()
    bot.get_channel = lambda cid: channel  # avoid any network fetch
    await bot.send_alert(_fake_meta(), None, reminder=reminder)
    assert len(channel.sends) == 1, "expected exactly one channel.send"
    return channel.sends[0]


async def test_checkpoint_alert_pings_authorized_user():
    print("test_checkpoint_alert_pings_authorized_user")
    sent = await _send_and_capture(reminder=False)
    check(sent["content"] == f"<@{AUTHORIZED_ID}>", "checkpoint content is the ID mention")
    check(sent["embed"] is not None and "Checkpoint" in sent["embed"].title,
          "embed still present and intact")


async def test_reminder_alert_pings_authorized_user():
    print("test_reminder_alert_pings_authorized_user")
    sent = await _send_and_capture(reminder=True)
    check(sent["content"] == f"<@{AUTHORIZED_ID}>", "reminder content is the ID mention")
    check(sent["embed"] is not None and "Reminder" in sent["embed"].title,
          "reminder embed still present and intact")


async def test_mention_is_in_content_not_embed():
    print("test_mention_is_in_content_not_embed")
    sent = await _send_and_capture(reminder=False)
    embed = sent["embed"]
    # The mention must live in content (which notifies), never leak into the embed.
    blob = " ".join(
        [embed.title or ""] + [f"{f.name} {f.value}" for f in embed.fields]
    )
    check(f"<@{AUTHORIZED_ID}>" not in blob, "mention does not appear inside the embed")


async def test_only_authorized_user_is_mentionable():
    print("test_only_authorized_user_is_mentionable")
    sent = await _send_and_capture(reminder=False)
    am = sent["allowed_mentions"]
    check(isinstance(am, discord.AllowedMentions), "allowed_mentions is set")
    check(am.everyone is False, "@everyone/@here suppressed")
    check(am.roles is False, "role mentions suppressed")
    ids = [getattr(u, "id", None) for u in (am.users or [])]
    check(ids == [int(AUTHORIZED_ID)], "only the authorized user id is mentionable")


async def main():
    for t in (
        test_checkpoint_alert_pings_authorized_user,
        test_reminder_alert_pings_authorized_user,
        test_mention_is_in_content_not_embed,
        test_only_authorized_user_is_mentionable,
    ):
        await t()
    print(f"\n{PASS} passed, {FAIL} failed")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
