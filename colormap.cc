#include "colormap.h"

#include <string_view>

#include "absl/strings/ascii.h"
#include "absl/strings/match.h"

const uint8_t* GetColormapFromName(std::string_view name) {
  name = absl::StripAsciiWhitespace(name);
  if (name == "15") return kColormap15;
  if (name == "17") return kColormap17;
  if (name == "7") return kColormap7;
  if (name == "92") return kColormap92;
  if (name == "85") return kColormap85;
  if (absl::EqualsIgnoreCase(name, "greyscale")) return kColormapGreyscale;
  if (absl::EqualsIgnoreCase(name, "grey")) return kColormapGrey;
  if (absl::EqualsIgnoreCase(name, "iron2")) return kColormapIron2;
  if (absl::EqualsIgnoreCase(name, "ironblack")) return kColormapIronBlack;
  if (absl::EqualsIgnoreCase(name, "rainbow")) return kColormapRainbow;
  return kColormapGrey;
}
