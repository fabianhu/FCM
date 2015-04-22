<nul set /p ".=#define VERSION_GIT 0x" > src\version-auto.h
git rev-parse --short=8 HEAD >> src\version-auto.h