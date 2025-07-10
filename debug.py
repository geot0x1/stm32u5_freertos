import subprocess
import time
import os

# --- CONFIG ---
openocd_path = "C:\\Users\\George\\Desktop\\openocd-cbc32c3-i686-w64-mingw32\\bin\\openocd.exe"
scripts_path = r"C:\Users\George\Desktop\openocd-cbc32c3-i686-w64-mingw32\share\openocd\scripts"
interface_cfg = "interface/stlink.cfg"
target_cfg = "target/stm32u5x.cfg"
elf_file = "build/stmtest.elf"
gdb_exe = "arm-none-eabi-gdb"

# Optional: auto GDB commands
gdb_commands = [
    "target remote localhost:3333",
    "monitor reset halt",
    f"load",
    "break main",
    "continue"
]

def launch_openocd():
    print("[*] Launching OpenOCD...")
    return subprocess.Popen(
        [openocd_path,
         "-s", scripts_path,
         "-f", interface_cfg,
         "-f", target_cfg],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

def launch_gdb():
    print("[*] Starting GDB...")
    gdb_cmds = "\n".join(gdb_commands)
    with open("gdb_commands.txt", "w") as f:
        f.write(gdb_cmds)

    subprocess.run([
        gdb_exe,
        elf_file,
        "-x", "gdb_commands.txt"
    ])

def main():
    # Start OpenOCD
    openocd_proc = launch_openocd()
    
    # Wait a bit for OpenOCD to initialize
    time.sleep(2)

    try:
        # Start GDB and connect
        launch_gdb()
    finally:
        print("[*] Shutting down OpenOCD...")
        openocd_proc.terminate()
        try:
            openocd_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            openocd_proc.kill()

if __name__ == "__main__":
    main()
