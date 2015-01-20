@echo off

del .\tagsearchfile.txt

for /r "C:\Texas Instruments\BLE-CC254x-1.4.0\Components\" %%g in (*.h) do echo %%g >> .\tagsearchfile.txt
for /r "C:\Texas Instruments\BLE-CC254x-1.4.0\Components\" %%g in (*.c) do echo %%g >> .\tagsearchfile.txt
for /r ".\" %%g in (*.c) do echo %%g >> .\tagsearchfile.txt
for /r ".\" %%g in (*.h) do echo %%g >> .\tagsearchfile.txt
for /r "..\Profiles" %%g in (*.c) do echo %%g >> .\tagsearchfile.txt
for /r "..\Profiles" %%g in (*.h) do echo %%g >> .\tagsearchfile.txt

ctags -R . -L tagsearchfile.txt

