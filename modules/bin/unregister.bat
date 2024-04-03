@echo off
cd /d %~dp0
echo Unregistering Tai-Ji OPCSim
@echo on
taiji-opcsim-server.exe -u
@echo off
pause