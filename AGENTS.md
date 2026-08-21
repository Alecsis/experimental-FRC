## Optional Graphify guidance

Graphify is optional development tooling. The repository must remain usable without Graphify installed and without local `graphify-out/` artifacts.

When a local `graphify-out/graph.json` exists, Graphify queries may be used as a navigation aid:

- Establish that the graph was generated from the current commit and relevant working-tree state before relying on it. If freshness cannot be established, treat the graph as stale.
- Use `graphify query "<question>"` for scoped exploration, `graphify path "<A>" "<B>"` for relationships, and `graphify explain "<concept>"` for focused concepts.
- Treat every graph-derived architectural claim as a lead to verify against the current tracked source, build configuration, tests, and other primary project files.
- The primary project files are authoritative whenever they disagree with graph output.
- Do not assume `graphify-out/`, reports, or visualizations are committed, present, current, or intended for version control.
- Only regenerate or update a graph when Graphify is available and the user or active workflow calls for it.

The committed `.graphifyignore`, when present, records the default scope used for optional application-focused graph generation. Review that scope when an analysis is intended to include documentation or auxiliary tooling.
