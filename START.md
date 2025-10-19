# ⚠️ Before Using the Makefile

🔹 **Before executing any Makefile command, ensure you have installed `arduino-cli`.**


#### 1. Download Arduino CLI
- Go to the official release page:
  🔗 [Arduino CLI Releases](https://github.com/arduino/arduino-cli/releases)
- Download the **ZIP file** for Windows (e.g., `arduino-cli_latest_Windows_64bit.zip`).
- Extract the contents to a folder, such as `C:\arduino-cli`.

---

#### 2. Add Arduino CLI to the System PATH (Optional, but Recommended)
If you want to use `arduino-cli` from any directory, add it to the **system PATH**:

1. Press `Win + R`, type `sysdm.cpl`, and press `Enter`.
2. Go to the **Advanced** tab → click **Environment Variables**.
3. In the **System Variables** section, find `Path` and click **Edit**.
4. Click **New** and enter the path to the folder where you extracted Arduino CLI (e.g., `C:\arduino-cli`).
5. Click **OK** to save the changes.

---

####  3. Verify the Installation
Open **Command Prompt (cmd)** and type:
```bash
arduino-cli version
```


Before using the Makefile, ensure that you have installed the correct rp2040 core. You can do this by running the following command in your command prompt:

```bash
arduino-cli config init
arduino-cli config add board_manager.additional_urls https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index
arduino-cli core install rp2040:rp2040
```

Additionally, to run the Makefile on Windows, you need to install MinGW-w64. MinGW-w64 provides a complete toolchain including the GNU Compiler Collection (GCC) and other essential tools for compiling code on Windows. You can download it from the official [ MinGW-w64 website](https://github.com/niXman/mingw-builds-binaries/releases).

Once downloaded, add the folder
`C:\...\mingw64\bin`
to your environment variables as a new PATH entry.


Next, you need to install GnuWin32 Make from a new cmd

```bash
winget install GnuWin32.Make
```

Next,you need to install Powershell from cmd

```bash
winget install Microsoft.Powershell --source winget
```

Then restart the PC


To verify that the installation of MinGW-w64 was successful, open a new command prompt and type:

```bash
gcc --version
```

To verify that the installation of GnuWin32 was successful, open a new command prompt and run:

```bash
make --version
```
If there are no errors and the respective versions of both tools are displayed, proceed with the installation of the libraries.

Regarding the libraries, you can install the necessary ones using Arduino CLI with the following commands.


```bash
arduino-cli lib update-index
arduino-cli lib install "Adafruit GFX Library"
arduino-cli lib install "Adafruit SH110X"
```
To test that everything is working correctly, open a terminal in Visual Studio and run "make help". If there are no errors, you can run "make compile" to compile.

# Guide to using the Makefile
The guide to using the Makefile is available inside the repository MAKEFILE-ARDUINO-CLI, accessible at the following link.
https://github.com/Team-Isaac-Polito/MAKEFILE-ARDUINO-CLI