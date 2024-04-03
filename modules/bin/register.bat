@echo off
cd /d %~dp0
echo Registering Tai-Ji OPCSim
@echo on
taiji-opcsim-server.exe -r
@echo off
pause