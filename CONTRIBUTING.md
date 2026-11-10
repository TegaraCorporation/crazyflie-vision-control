# Contributing

Thank you for looking at this project. It is a learning environment, so clear and simple code matters more than clever code.

Before you open a pull request, please check these points.

1. Say what you flew or tested. Changes to the flight scripts need a short note on the hardware, the battery voltage, and what the log showed. Changes that were only tested offline should say so.
2. Keep the safety behaviour. Marker loss, battery, area limit, time limit and the emergency stop must keep working. A change that removes or loosens one of them needs a clear reason.
3. Keep the code readable for students. Plain Python, comments that explain why, and the settings block at the top of each script.
4. Update the guide in `docs/` when a command, a setting or a result changes. The guide exists in English and Japanese. If you can only update one, say so in the pull request.
5. Do not commit calibration files, flight logs or your own `phase2_settings.json`. They belong to one setup.

To report a problem, open an issue with the script name, the full console output, your operating system, Python version and `pip show cflib`.

By contributing you agree that your contribution is released under the GNU General Public License, version 3 or later, the same as the rest of the project.
