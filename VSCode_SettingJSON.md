# Enable "conda activate" in Powershell of VS Code
# Ctrl + Shift + P -> Preferences: Open Settings (JSON)
{
    "workbench.colorTheme": "Default Dark+",
    "security.workspace.trust.untrustedFiles": "open",
    "terminal.explorerKind": "external",
    "terminal.integrated.defaultProfile.windows": "PowerShell",
    "terminal.integrated.profiles.windows": {
        "PowerShell":{
          "source": "PowerShell",
          "args": [
            "-ExecutionPolicy",
            "ByPass",
            "-NoExit",
            "-Command",
            "& C:\\ProgramData\\Anaconda3\\shell\\condabin\\conda-hook.ps1"
            ]
        }
      },
}
