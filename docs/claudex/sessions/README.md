# Session notes

Use one file per date: `YYYY-MM-DD.md`. If there are multiple sessions on one date, append a clearly titled section; never replace an existing session record.

These notes are an audit trail, not the project’s live instruction file:

- `/bootstrap` reads the latest note named by `claude.md`, then reads its `## Handoff` section.
- `/recap` writes evidence-backed state here and updates the root handoff pointer.
- `/tldr` reviews recent notes only to promote durable knowledge to the vault.
- Detailed chronology belongs here or in `docs/claudex/history.md`; do not copy it into `claude.md`.

Every active note ends with `## Handoff` containing only:

- current state
- verified facts
- unverified items
- next safe action
- things not to change

Older notes may remain detailed so measurements and decisions are recoverable. Treat superseded conclusions as historical unless the latest handoff or a current source/test re-establishes them.
