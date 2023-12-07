@echo off

:: Add all changes to the staging area
git add --all

:: remove folders
git rm --cached -r "MDDS Testing"/MDK-ARM/"MDDS Testing"
git rm --cached -r "Drohne Lendl"/MDK-ARM/"Drohne Lendl"
git rm --cached -r Sponsoring/Rechnungen
git rm --cached -r Praesentationen/"Tag der offenen Tuer"/MPU9150_DEMO_REJ/DEMO_MPU_9150_V2
git rm --cached -r Praesentationen/"Tag der offenen Tuer"/MPU9150_DEMO_REJ/Grove_IMU_9DOF_9250
git rm --cached -r Praesentationen/"Tag der offenen Tuer"/MPU9150_DEMO_REJ/MPU9250-master_kriswiner
git rm --cached -r Praesentationen/"Tag der offenen Tuer"/MPU9150_DEMO_REJ/HTL_Hollabrunn.STD.Pack.3.2.0.pack
git rm --cached -r *.html
git rm --cached -r *.htm

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