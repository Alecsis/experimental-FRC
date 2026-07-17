#!/usr/bin/env python3
import subprocess
import sys
import time

def run_headless_validation(timeout=12):
    print("🤖 [Subagent: Sim] Launching headless robot simulation...")
    
    # Run simulateJava with headless environment flags
    cmd = ["./gradlew", "simulateJava", "-Djava.awt.headless=true"]
    
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    
    start_time = time.time()
    errors = []
    state_machine_initialized = False
    
    try:
        while time.time() - start_time < timeout:
            line = process.stdout.readline()
            if line:
                clean_line = line.strip()
                print(f"  [SIM] {clean_line}")
                
                # Catch any unhandled state machine or dashboard errors
                if "Exception" in clean_line or "Error" in clean_line:
                    errors.append(clean_line)
                
                if "********** Robot program starting **********" in clean_line:
                    state_machine_initialized = True
                    
            if process.poll() is not None:
                break
    finally:
        process.terminate()
        process.wait()

    print("\n--- 🏁 Sim Subagent Report ---")
    if errors:
        print("❌ CRITICAL: Runtime exceptions detected during execution:")
        for err in errors:
            print(f"  -> {err}")
        sys.exit(1)
        
    if not state_machine_initialized:
        print("❌ CRITICAL: Robot failed to reach initialization loop.")
        sys.exit(1)
        
    print("✅ SUCCESS: State machine initialized with 0 runtime errors.")
    sys.exit(0)

if __name__ == "__main__":
    run_headless_validation()