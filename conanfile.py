from conans import ConanFile, CMake, tools 

class Spline(ConanFile):
  name = "Spline"
  version = "0.0.1"
  license = "<Put the package license here>"
  description = "Spline curve fitter"
  settings = "os", "compiler", "build_type", "arch"
  options = {"shared": [True, False], "compiler": ["gcc", "clang"]}
  default_options = {"shared": False, "compiler": "gcc"}
  generators = "cmake", "cmake_find_package"
     
  def export_sources(self):
    self.copy("*.cpp", dst="src", src="src")
    self.copy("CMakeLists.txt", src="src", dst="src")
    self.copy("*.hpp", dst="include", src="include")
    self.copy("*.h", dst="include", src="include")
    self.copy("*.h", dst="include", src="include_")

  def build(self):
    cmake = CMake(self)
    cmake.configure(source_folder="src")
    cmake.build()

  def package(self):
    self.copy("*.h", dst="include", src="include")
    self.copy("*.hpp", dst="include", src="include")
    self.copy("*.h", dst="include", src="include_")
    self.copy("*.lib", dst="lib", keep_path=False)
    self.copy("*.dll", dst="bin", keep_path=False)
    self.copy("*.dylib", dst="lib", keep_path=False)
    self.copy("*.so", dst="lib", keep_path=False)
    self.copy("*.a", dst="lib", keep_path=False)

  def package_info(self):
    self.cpp_info.libs = ["Spline"]
