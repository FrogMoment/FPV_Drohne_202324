@echo off

:: Add all changes to the staging area
git add --all

:: remove folders
git rm --cached -r "MDDS Testing"/MDK-ARM/"MDDS Testing"
git rm --cached -r "Drohne Lendl"/MDK-ARM/"Drohne Lendl"
git rm --cached -r Rechnungen

::show status
git status

:: Prompt for the commit message
set /p commit_message="Enter your commit message: "

:: Commit the changes with the provided commit message
git commit -m "%commit_message%"

:: Push to the master branch
git push origin master

:: Pause to view the output (you can remove this line if not needed)
pause