set theCurrentDate to current date
set dateTime to short date string of theCurrentDate & space & time string of theCurrentDate
set P to offset of "/" in dateTime
set dateTime to text 1 through (P - 1) of dateTime & "-" & text (P + 1) through -1 of dateTime
set P to offset of "/" in dateTime
set dateTime to text 1 through (P - 1) of dateTime & "-" & text (P + 1) through -1 of dateTime
set theFilePath to "/Users/junhyeokahn/Repository/PnC/ExperimentVideo/" & dateTime & ".mov"


tell application "QuickTime Player"
    set newMovieRecording to new movie recording
    tell newMovieRecording
        start
        tell application "Finder"
            display dialog "Save video?" default answer "yes"
            set save_answer to text returned of result
            if (save_answer = "yes") then

                tell application "QuickTime Player"
                    pause
                    save newMovieRecording in POSIX file theFilePath
                    stop
                    close newMovieRecording
                end tell
            end if
        end tell
    end tell
end tell
