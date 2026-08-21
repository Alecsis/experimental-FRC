# Session notes

Use one file per date: `YYYY-MM-DD.md`. If there are multiple sessions on one date, append a clearly titled section; never replace an existing session record.

These notes are an audit trail, not the project’s live instruction file:

- `/bootstrap` reads the latest note named by `claude.md`, then reads its `## Handoff` section. When a file holds several same-day sessions it holds several `## Handoff` sections: **the last one in the file wins.** Mark every earlier one `## Handoff (SUPERSEDED — ...)` when you append a new session, so a reader landing on the first cannot mistake it for current. Do not edit a superseded handoff's body — it was true when written.
- `/recap` writes evidence-backed state here and updates the root handoff pointer.
- `/tldr` reviews recent notes only to promote durable knowledge to the vault.
- Detailed chronology belongs here or in `docs/claudex/history.md`; do not copy it into `claude.md`.

Every active note ends with `## Handoff` containing only:

- current state
- verified facts
- unverified items
- next safe action
- things not to change

State the current position by *content milestone* (the commit that changed source, what it did), not
by the file's own tip SHA. A handoff cannot pin the SHA of the commit that writes it — recording the
value and creating it are the same event, so the number is stale the instant it lands. Point at
`git log --oneline -5` / `git status -sb` for the tip instead.

Older notes may remain detailed so measurements and decisions are recoverable. Treat superseded conclusions as historical unless the latest handoff or a current source/test re-establishes them.
