cd ..
cd src
del version-auto.h
echo //autogenerated file > version-auto.h
<nul set /p ".=#define VERSION_GIT " >> version-auto.h
git rev-parse --short=8 HEAD >> version-auto.h
<nul set /p ".=#define VERSION_GITN 0x" >> version-auto.h
git rev-parse --short=8 HEAD >> version-auto.h
echo Version generated.