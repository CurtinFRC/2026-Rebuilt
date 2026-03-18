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

bool ValidateGroupSchemaNode(const nlohmann::json& node) {
  if (!node.is_object()) {
    return false;
  }
  const auto validateFieldArray = [&](const char* key) {
    const auto it = node.find(key);
    if (it == node.end()) {
      return true;
    }
    if (!it->is_array()) {
      return false;
    }
    for (const auto& field : *it) {
      if (!field.is_object()) {
        return false;
      }
    }
    return true;
  };
  return validateFieldArray("fields");
}

}  // namespace

bool ValidateSchemaSetFile(const std::string& path, std::string* error) {
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

  nlohmann::json root;
  try {
    in >> root;
  } catch (...) {
    if (error != nullptr) {
      *error = "failed to parse schema JSON";
    }
    return false;
  }
  if (!root.is_object()) {
    if (error != nullptr) {
      *error = "schema file root must be an object";
    }
    return false;
  }

  const auto validateGroupIfPresent = [&](const char* key) {
    const auto it = root.find(key);
    if (it == root.end()) {
      return true;
    }
    return ValidateGroupSchemaNode(*it);
  };

  if (!validateGroupIfPresent("fieldVision") ||
      !validateGroupIfPresent("repulsor") ||
      !validateGroupIfPresent("cameras")) {
    if (error != nullptr) {
      *error = "one or more fixed schema groups are invalid";
    }
    return false;
  }

  const auto channelsIt = root.find("channels");
  if (channelsIt != root.end()) {
    if (!channelsIt->is_array()) {
      if (error != nullptr) {
        *error = "channels must be an array";
      }
      return false;
    }
    for (const auto& channelNode : *channelsIt) {
      if (!channelNode.is_object()) {
        if (error != nullptr) {
          *error = "channels entries must be objects";
        }
        return false;
      }
      const std::string channelName = channelNode.value("channel", std::string{});
      if (channelName.empty()) {
        if (error != nullptr) {
          *error = "channel entry missing non-empty 'channel'";
        }
        return false;
      }
      if (!ValidateGroupSchemaNode(channelNode)) {
        if (error != nullptr) {
          *error = "channel schema invalid for '" + channelName + "'";
        }
        return false;
      }
    }
  }
  return true;
}

bool LoadSchemaSetFromFile(const std::string& path, NtSchemaSet& inOutSet, std::string* error) {
  if (path.empty()) {
    if (error != nullptr) {
      *error = "schema file path is empty";
    }
    return false;
  }

  std::string validationError;
  if (!ValidateSchemaSetFile(path, &validationError)) {
    if (error != nullptr) {
      *error = validationError;
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
