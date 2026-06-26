#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  std::filesystem::path config = "config/ureca_experiments.yaml";
  std::filesystem::path output = "results/E8_blackbox_wrapper_demo";

  for (int i = 1; i < argc; ++i)
  {
    const std::string token = argv[i];
    if (token == "--config" && i + 1 < argc)
      config = argv[++i];
    else if (token == "--output" && i + 1 < argc)
      output = argv[++i];
  }

  const std::string command =
      std::string("./ureca_run_experiment_suite --config ") + config.string() +
      " --experiment E8_blackbox_wrapper_demo --output " + output.string();
  return std::system(command.c_str());
}
