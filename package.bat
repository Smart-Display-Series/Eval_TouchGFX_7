@echo off

set current_dir=%cd%

rem
set tgfx_path=C:\TouchGFX\4.18.1\designer\

set source_project=Eval_TouchGFX_7

set version=1.0.0

set source_tpa=%source_project%-%version%.tpa 

set dest_path=C:\TouchGFX\4.18.1\app\packages\

rem echo %current_dir%

%tgfx_path%\tgfx.exe pack -d %source_project%

echo.

echo Please modify string of Version in json file

pause

%tgfx_path%\tgfx.exe pack -rc -d %source_project%

copy %source_tpa% %dest_path%

del %source_project%.zip

del %source_project%-%version%.zip



