@ECHO OFF

set path=%PATH%;C:\Program Files (x86)\Atmel\Atmel Studio 6.2\

REM get date and time
for /f "delims=" %%a in ('date/t') do @set mydate=%%a
for /f "delims=" %%a in ('time/t') do @set mytime=%%a
set fvar=%mydate%%mytime%

call git pull origin autobuild > pull.txt

REM get the length of pull.txt
for %%a in (pull.txt) do (
set length=%%~za
)

if %length% GTR 20 (
echo building... %fvar% 

START /WAIT atmelstudio.exe FabOS32.cproj /build debug /out autobuild.txt

call git commit -m "autobuild %fvar%"
call git push origin autobuild

echo finished... %mytime%
)

rem re-run script
TIMEOUT /T 60 /NOBREAK
fcmbuild.bat
