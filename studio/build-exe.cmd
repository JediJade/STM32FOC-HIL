@echo off
rem Build HIL.exe - one self-contained file, no Python needed to run it.
rem
rem Output:  studio\dist\HIL.exe
rem Scratch: studio\dist\work  (safe to delete)
rem
rem Needs Python 3 with Pillow and PyInstaller:  pip install pillow pyinstaller
rem
rem Paths are absolute because --specpath makes PyInstaller resolve
rem --add-data relative to the spec file, not to this directory.
rem
rem   HIL.exe                    the window, asking for ports
rem   HIL.exe --model            no hardware: the same window on made-up data
rem   HIL.exe --hil COM5 --drive COM4

setlocal
cd /d "%~dp0"

python -m PyInstaller ^
  --onefile ^
  --windowed ^
  --name HIL ^
  --icon "%~dp0..\unitymbed.ico" ^
  --add-data "%~dp0..\unitymbed.ico;." ^
  --exclude-module numpy ^
  --distpath dist ^
  --workpath dist\work ^
  --specpath dist ^
  --noconfirm ^
  --clean ^
  hil.py

if errorlevel 1 (
  echo.
  echo build failed
  exit /b 1
)

echo.
echo built dist\HIL.exe
