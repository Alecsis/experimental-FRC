# CLAUDE.md - FRC Hybrid Refactor Guidelines

## 🤖 System Context
- **Project Type:** FRC Robotics Code (2026/2027 Transition)
- **User Role:** Lead Programmer/Mentor overseeing the system architecture.
- **Goal:** Build a modular, simulation-first codebase combining 1678's IO-isolation and 254's state machine.

## 🛠️ Superpowers (Available MCP Tools & Plugins)
- **Claude-Mem:** Core context & lifecycle memory loop tracker.
- **Context7:** Up-to-date WPILib & JavaDocs API reference layer.
- **Gemini MCP:** Deep semantic workspace/repository analysis.
- **Tavily/DuckDuckGo:** Real-time web-search validation engine.
- **Fetch:** Remote markdown/URL scraping capability.

## 📂 Reference Directories (`temp_reference/`)
You have read access to elite architectural templates inside these subdirectories. Use them for structural examples, but never merge or import them directly:
- `temp_reference/Lynk 2026/`
- `temp_reference/Phoenix 6 API Examples/`
- `temp_reference/Team 254 Code/`
- `temp_reference/Team 1678 Code/`
- `temp_reference/Team 6328 Code/`

## 🛠️ Build & Test Commands
- **Compile Java:** `./gradlew compileJava`
- **Build Robot:** `./gradlew build`
- **Run Simulator:** `./gradlew simulateJava`

## 📐 Architecture Rules
1. **Strict Hardware Isolation:** No vendor hardware APIs (CTRE Phoenix, REVLib, etc.) are allowed in standard subsystem files. They must live strictly inside `*IOReal.java` implementations.
2. **Singleton Subsystems:** Every subsystem and the `Superstructure` must implement the Singleton pattern (`private` constructor and `public static Subsystem getInstance()`).
3. **Centralized States:** The `Superstructure` coordinates complex multi-subsystem states. Subsystems themselves only handle their immediate mechanism control.

## 🧠 Karpathy Guidelines (Anti-Failure Mode Protocol)
To ensure elite execution, strictly adhere to these behaviors:
- **No Assumptions:** Never guess vendor API syntax or class constructor shapes (especially Phoenix 6 vs 5). Stop and use `context7` or web search to verify the real-world signature first.
- **Simplicity First (Anti-Overengineering):** Write the cleanest, most minimalist code required to satisfy the immediate structural requirement or simulation goal.
- **Surgical Changes Only:** Modify *only* the specific files related to the active task. Do not rewrite, clean up, or change formatting in adjacent methods or unrelated classes.
- **Never Declare Success Early:** A task is not complete until you explicitly run `./gradlew compileJava` via the shell tool and confirm a zero-error output. If it breaks, fix it immediately.