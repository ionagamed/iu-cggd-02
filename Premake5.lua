workspace "Basics of ray tracing"
   configurations { "Debug", "Release" }
   language "C++"
   cppdialect "C++17"
   architecture "x64"
   systemversion "latest"
   toolset "v142"
   optimize "Speed"
   buildoptions { "/openmp" }
   filter "configurations:Debug"
      defines { "DEBUG" }
      symbols "On"

   filter "configurations:Release"
      defines { "NDEBUG" }
      optimize "On"

   targetdir ("bin/%{prj.name}/%{cfg.longname}")
   objdir ("obj/%{prj.name}/%{cfg.longname}")
    
project "06. Ray generation"
   kind "ConsoleApp"
   includedirs { "lib/stb" }
   includedirs { "lib/linalg" }
   includedirs { "src/" }
   files {"src/ray_generation.h", "src/ray_generation.cpp" }
   files {"src/ray_generation_main.cpp"}

project "07. Moller-Trumbore algorithm"
   kind "ConsoleApp"
   includedirs { "lib/stb" }
   includedirs { "lib/linalg" }
   includedirs { "src/" }
   files {"src/ray_generation.h", "src/ray_generation.cpp" }
   files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
   files {"src/mt_algorithm_main.cpp"}

project "08. Lighting and shadows"
   kind "ConsoleApp"
   includedirs { "lib/stb" }
   includedirs { "lib/linalg" }
   includedirs { "lib/tinyobjloader" }
   includedirs { "src/" }
   files {"src/ray_generation.h", "src/ray_generation.cpp" }
   files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
   files {"src/lighting_and_shadows.h", "src/lighting_and_shadows.cpp"}
   files {"src/lighting_and_shadows_main.cpp"}

project "09. Acceleration structures"
   kind "ConsoleApp"
   includedirs { "lib/stb" }
   includedirs { "lib/linalg" }
   includedirs { "lib/tinyobjloader" }
   includedirs { "src/" }
   files {"src/ray_generation.h", "src/ray_generation.cpp" }
   files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
   files {"src/lighting_and_shadows.h", "src/lighting_and_shadows.cpp"}
   files {"src/acceleration_structures.h", "src/acceleration_structures.cpp"}
   files {"src/acceleration_structures_main.cpp"}


project "10. Anti-aliasing"
   kind "ConsoleApp"
   includedirs { "lib/stb" }
   includedirs { "lib/linalg" }
   includedirs { "lib/tinyobjloader" }
   includedirs { "src/" }
   files {"src/ray_generation.h", "src/ray_generation.cpp" }
   files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
   files {"src/lighting_and_shadows.h", "src/lighting_and_shadows.cpp"}
   files {"src/acceleration_structures.h", "src/acceleration_structures.cpp"}
   files {"src/anti_aliasing.h", "src/anti_aliasing.cpp"}
   files {"src/anti_aliasing_main.cpp"}