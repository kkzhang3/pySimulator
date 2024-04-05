
set "searchKey=HKEY_LOCAL_MACHINE\SOFTWARE\WOW6432Node\TaiJiControl"
set "valueName=InstallPath"

echo Searching for value %valueName% in registry key: %searchKey%

for /f "tokens=2,*" %%A in ('reg query "%searchKey%" /v "%valueName%" ^| findstr "%valueName%"') do (
    set "value=%%B\Python312"
)
cd "%value%"
start pythonw.exe "%~dp0main.py"
