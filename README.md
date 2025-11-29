# _Sample project_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This is the simplest buildable example. The example is used by command `idf.py create-project`
that copies the project to user specified path and set it's name. For more information follow the [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project)



## How to use example
We encourage the users to use the example as a template for the new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.

## Android app

This repository also contains an Android application in the `android-app/` folder. The Android project is included for convenience and should be built from Android Studio or the Gradle wrapper.

- Do NOT commit `local.properties` or keystore files (`*.jks`, `*.keystore`). A project-level `.gitignore` is present to prevent accidental commits of these files.
- To build locally, create `local.properties` (not checked in) containing your Android SDK path, for example:

```
sdk.dir=C:/Users/isaac/AppData/Local/Android/Sdk
```

- Open the `android-app` folder in Android Studio or run the Gradle wrapper from the `android-app` folder:

```powershell
cd android-app
.\gradlew assembleDebug
```

- The repo-level `.gitignore` ignores `android-app/build/` and other generated files.
