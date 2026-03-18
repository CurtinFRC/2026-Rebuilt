#if defined(REPULSOR_HAS_NTCORE)

#include "repulsor3d/nt/SchemaFileLoader.hpp"

#include <fstream>
#include <utility>

#include <nlohmann/json.hpp>

namespace repulsor3d::nt {
namespace {

::nt::NetworkTableType ParseType(const std::string& type) {
  if (type == "boolean" || type == "bool") {
    return ::nt::NetworkTableType::kBoolean;
  }
  if (type == "string") {
    return ::nt::NetworkTableType::kString;
  }
  return ::nt::NetworkTableType::kDouble;
}

void ReadFieldSpec(const nlohmann::json& node, ScalarFieldSpec& outSpec) {
  outSpec.key = node.value("key", outSpec.key);
  outSpec.topicSuffix = node.value("topicSuffix", outSpec.topicSuffix);
  outSpec.type = ParseType(node.value("type", std::string("double")));
  outSpec.defaultDouble = node.value("defaultDouble", outSpec.defaultDouble);
  outSpec.defaultBoolean = node.value("defaultBoolean", outSpec.defaultBoolean);
  outSpec.defaultString = node.value("defaultString", outSpec.defaultString);
}

void ReadGroupSchema(const nlohmann::json& node, EntityGroupSchema& outSchema) {
  outSchema.tablePrefix = node.value("tablePrefix", outSchema.tablePrefix);
  outSchema.idPrefix = node.value("idPrefix", outSchema.idPrefix);
  outSchema.autoCaptureAdditionalScalars =
      node.value("autoCaptureAdditionalScalars", outSchema.autoCaptureAdditionalScalars);
  outSchema.extraKeyPrefix = node.value("extraKeyPrefix", outSchema.extraKeyPrefix);

  const auto aliveIt = node.find("aliveField");
  if (aliveIt != node.end() && aliveIt->is_object()) {
    ReadFieldSpec(*aliveIt, outSchema.aliveField);
  }

  const auto fieldsIt = node.find("fields");
  if (fieldsIt != node.end() && fieldsIt->is_array()) {
    outSchema.fields.clear();
    for (const auto& fieldNode : *fieldsIt) {
      if (!fieldNode.is_object()) {
        continue;
      }
      ScalarFieldSpec spec;
      ReadFieldSpec(fieldNode, spec);
      outSchema.fields.push_back(std::move(spec));
    }
  }
}

bool ReadDynamicChannel(const nlohmann::json& node, NtSchemaSet::DynamicChannelSchema& out) {
  if (!node.is_object()) {
    return false;
  }
  const std::string channel = node.value("channel", std::string{});
  if (channel.empty()) {
    return false;
  }

  out.channel = channel;
  ReadGroupSchema(node, out.schema);
  return true;
}

}  // namespace

bool LoadSchemaSetFromFile(const std::string& path, NtSchemaSet& inOutSet, std::string* error) {
  if (path.empty()) {
    if (error != nullptr) {
      *error = "schema file path is empty";
    }
    return false;
  }

  std::ifstream in(path);
  if (!in.is_open()) {
    if (error != nullptr) {
      *error = "failed to open schema file: " + path;
    }
    return false;
  }

  try {
    const nlohmann::json root = nlohmann::json::parse(in);
    if (!root.is_object()) {
      if (error != nullptr) {
        *error = "schema file root must be an object";
      }
      return false;
    }

    if (const auto it = root.find("fieldVision"); it != root.end() && it->is_object()) {
      ReadGroupSchema(*it, inOutSet.fieldVision);
    }
    if (const auto it = root.find("repulsor"); it != root.end() && it->is_object()) {
      ReadGroupSchema(*it, inOutSet.repulsor);
    }
    if (const auto it = root.find("cameras"); it != root.end() && it->is_object()) {
      ReadGroupSchema(*it, inOutSet.cameras);
    }
    if (const auto it = root.find("channels"); it != root.end() && it->is_array()) {
      inOutSet.dynamicChannels.clear();
      inOutSet.dynamicChannels.reserve(it->size());
      for (const auto& channelNode : *it) {
        NtSchemaSet::DynamicChannelSchema channelSchema;
        if (ReadDynamicChannel(channelNode, channelSchema)) {
          inOutSet.dynamicChannels.push_back(std::move(channelSchema));
        }
      }
    }

    return true;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  }
}

}  // namespace repulsor3d::nt

#endif  // defined(REPULSOR_HAS_NTCORE)
