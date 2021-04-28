For more information or feedback contact yifuluo@u.northwestern.edu

## Build using CMake (Windows OS):
1. Install CMake-gui [here](https://cmake.org/download/).

2. Clone this repository:

```
	git clone https://github.com/SeNSE-lab/MuscleModel.git
```

3. Clone Bullet3 (an open-source physics engine) into this repo directory

```
	git clone https://github.com/bulletphysics/bullet3
```

4. Compile Bullet first.

   Run CMake-gui.exe. 

   Select source code directory as "{your_path}/MuscleModel/bullet3".

   Select binaries directory as "{your_path}/MuscleModel/bullet3/build" (select "yes" when prompted asking to create build directory).
   
   **Specify "LIBRARY_OUTPUT_PATH" field value to "{your_path}/MuscleModel/bullet3/build/lib" if it is not so already.**

   Click **Configure**, then **Generate**, select your version of visual studio as the generator.

   Double-click open "MuscleModel/bullet3-master/build/BULLET_PHYSICS.sln" using Microsoft Visual Studio.

   In the Solution Explorere, right click "ALL_BUILD" to compile Bullet physics. This may take a while.

   When finished, "File->Close solution", then exit Visual Studio.

5. Compile MuscleModel

   Run CMake-gui.exe again.

   Select source code directory as "{your_path}/MuscleModel".

   Select binaries directory as "{your_path}/MuscleModel/build" (select "yes" when prompted asking to create build directory).

   Click **Configure**, then **Generate**, select your version of visual studio as the generator.

   Double-click open "MuscleModel/build/Apps_MuscleModel.sln" using Microsoft Visual Studio.

   In the Solution Explorere, right click "ALL_BUILD" to compile MuscleModel.

   
