#include "repulsor3d/render/CadModelRenderFeature.hpp"

#include <GL/glew.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>

#include "repulsor3d/render/cad/CadShaderSources.hpp"

namespace repulsor3d {

bool CadModelRenderFeature::CreateShader() {
  const char* vsSrc = cad::CadMainVertexShaderSource();
  const char* fsSrc = cad::CadMainFragmentShaderSource();

  const unsigned int vs = CompileShader(GL_VERTEX_SHADER, vsSrc);
  const unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fsSrc);
  if (vs == 0 || fs == 0) {
    if (vs != 0) {
      glDeleteShader(vs);
    }
    if (fs != 0) {
      glDeleteShader(fs);
    }
    return false;
  }

  if (!LinkShader(shader_, vs, fs)) {
    glDeleteShader(vs);
    glDeleteShader(fs);
    return false;
  }

  glDeleteShader(vs);
  glDeleteShader(fs);

  uMvpLoc_ = backend_->GetUniformLocation(shader_.Get(), "uViewProjection");
  uModelLoc_ = -1;
  uNormalMatrixLoc_ = -1;
  uColorLoc_ = backend_->GetUniformLocation(shader_.Get(), "uColor");
  uLightDirLoc_ = backend_->GetUniformLocation(shader_.Get(), "uLightDir");
  uCameraPosLoc_ = backend_->GetUniformLocation(shader_.Get(), "uCameraPos");
  uKeyLightIntensityLoc_ = backend_->GetUniformLocation(shader_.Get(), "uKeyLightIntensity");
  uFillLightIntensityLoc_ = backend_->GetUniformLocation(shader_.Get(), "uFillLightIntensity");
  uAmbientStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uAmbientStrength");
  uDepthCueStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uDepthCueStrength");
  uSpecularStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uSpecularStrength");
  uRimStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uRimStrength");
  uRoughnessLoc_ = backend_->GetUniformLocation(shader_.Get(), "uRoughness");
  uMetallicLoc_ = backend_->GetUniformLocation(shader_.Get(), "uMetallic");
  uExposureLoc_ = backend_->GetUniformLocation(shader_.Get(), "uExposure");
  uFogDensityLoc_ = backend_->GetUniformLocation(shader_.Get(), "uFogDensity");
  uSaturationLoc_ = backend_->GetUniformLocation(shader_.Get(), "uSaturation");
  uGammaLoc_ = backend_->GetUniformLocation(shader_.Get(), "uGamma");
  uUseAssetColorLoc_ = backend_->GetUniformLocation(shader_.Get(), "uUseAssetColor");
  uLightViewProjectionLoc_ = backend_->GetUniformLocation(shader_.Get(), "uLightViewProjection");
  uLightViewProjectionFarLoc_ = backend_->GetUniformLocation(shader_.Get(), "uLightViewProjectionFar");
  uShadowMapLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadowMap");
  uShadowMapFarLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadowMapFar");
  uShadowEnabledLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadowEnabled");
  uShadowStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadowStrength");
  uShadowPcfRadiusLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadowPcfRadius");
  uShadowCascadeCountLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadowCascadeCount");
  uShadowCascadeSplitMLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadowCascadeSplitM");
  uAlbedoMapLoc_ = backend_->GetUniformLocation(shader_.Get(), "uAlbedoMap");
  uNormalMapLoc_ = backend_->GetUniformLocation(shader_.Get(), "uNormalMap");
  uHasAlbedoMapLoc_ = backend_->GetUniformLocation(shader_.Get(), "uHasAlbedoMap");
  uHasNormalMapLoc_ = backend_->GetUniformLocation(shader_.Get(), "uHasNormalMap");
  uShadingModeLoc_ = backend_->GetUniformLocation(shader_.Get(), "uShadingMode");
  uNormalStrengthLoc_ = backend_->GetUniformLocation(shader_.Get(), "uNormalStrength");
  uTriplanarScaleLoc_ = backend_->GetUniformLocation(shader_.Get(), "uTriplanarScale");

  constexpr const char* shadowVsSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 3) in vec4 iModelRow0;
layout(location = 4) in vec4 iModelRow1;
layout(location = 5) in vec4 iModelRow2;
layout(location = 6) in vec4 iModelRow3;
uniform mat4 uLightViewProjection;
void main() {
  mat4 model = mat4(iModelRow0, iModelRow1, iModelRow2, iModelRow3);
  gl_Position = uLightViewProjection * model * vec4(aPos, 1.0);
}
)";
  constexpr const char* shadowFsSrc = R"(
#version 330 core
void main() {}
)";
  const unsigned int shadowVs = CompileShader(GL_VERTEX_SHADER, shadowVsSrc);
  const unsigned int shadowFs = CompileShader(GL_FRAGMENT_SHADER, shadowFsSrc);
  if (shadowVs == 0 || shadowFs == 0) {
    if (shadowVs != 0) {
      glDeleteShader(shadowVs);
    }
    if (shadowFs != 0) {
      glDeleteShader(shadowFs);
    }
    return false;
  }
  if (!LinkShader(shadowShader_, shadowVs, shadowFs)) {
    glDeleteShader(shadowVs);
    glDeleteShader(shadowFs);
    return false;
  }
  glDeleteShader(shadowVs);
  glDeleteShader(shadowFs);
  uShadowLightMvpLoc_ = backend_->GetUniformLocation(shadowShader_.Get(), "uLightViewProjection");

  if (shadowEnabled_) {
    auto createShadowTarget = [&](GlTextureHandle& textureHandle, unsigned int& fbo) -> bool {
      const unsigned int tex = backend_->CreateTexture2D();
      textureHandle.Set(tex);
      glBindTexture(GL_TEXTURE_2D, textureHandle.Get());
      glTexImage2D(
          GL_TEXTURE_2D,
          0,
          GL_DEPTH_COMPONENT24,
          shadowMapSize_,
          shadowMapSize_,
          0,
          GL_DEPTH_COMPONENT,
          GL_FLOAT,
          nullptr);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
      const float border[] = {1.0F, 1.0F, 1.0F, 1.0F};
      glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

      glGenFramebuffers(1, &fbo);
      glBindFramebuffer(GL_FRAMEBUFFER, fbo);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, textureHandle.Get(), 0);
      glDrawBuffer(GL_NONE);
      glReadBuffer(GL_NONE);
      if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
      }
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      return true;
    };

    if (!createShadowTarget(shadowDepthTexture_, shadowFbo_)) {
      return false;
    }
    if (shadowCascadeCount_ > 1) {
      if (!createShadowTarget(shadowDepthTextureFar_, shadowFboFar_)) {
        return false;
      }
    } else {
      shadowDepthTextureFar_.Reset();
      if (shadowFboFar_ != 0) {
        glDeleteFramebuffers(1, &shadowFboFar_);
        shadowFboFar_ = 0;
      }
    }
  }

  materialPipeline_.AddPass(std::make_unique<FlatLitMaterialPass>(*backend_, uColorLoc_));
  materialPipeline_.AddPass(std::make_unique<WireframeMaterialPass>(*backend_));
  return uMvpLoc_ >= 0 &&
         uColorLoc_ >= 0 && uLightDirLoc_ >= 0 && uCameraPosLoc_ >= 0 &&
         uKeyLightIntensityLoc_ >= 0 && uFillLightIntensityLoc_ >= 0 &&
         uAmbientStrengthLoc_ >= 0 &&
         uDepthCueStrengthLoc_ >= 0 && uSpecularStrengthLoc_ >= 0 &&
         uRimStrengthLoc_ >= 0 && uRoughnessLoc_ >= 0 && uMetallicLoc_ >= 0 &&
         uExposureLoc_ >= 0 && uFogDensityLoc_ >= 0 && uSaturationLoc_ >= 0 &&
         uGammaLoc_ >= 0 && uUseAssetColorLoc_ >= 0 &&
         uLightViewProjectionLoc_ >= 0 && uLightViewProjectionFarLoc_ >= 0 &&
         uShadowMapLoc_ >= 0 &&
         uShadowMapFarLoc_ >= 0 &&
         uShadowEnabledLoc_ >= 0 && uShadowStrengthLoc_ >= 0 &&
         uShadowPcfRadiusLoc_ >= 0 && uShadowCascadeCountLoc_ >= 0 && uShadowCascadeSplitMLoc_ >= 0 &&
         uAlbedoMapLoc_ >= 0 && uNormalMapLoc_ >= 0 &&
         uHasAlbedoMapLoc_ >= 0 && uHasNormalMapLoc_ >= 0 && uShadingModeLoc_ >= 0 &&
         uNormalStrengthLoc_ >= 0 && uTriplanarScaleLoc_ >= 0 &&
         uShadowLightMvpLoc_ >= 0;
}

unsigned int CadModelRenderFeature::CompileShader(const unsigned int type, const char* source) {
  const unsigned int shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);

  int ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (ok == GL_TRUE) {
    return shader;
  }

  int length = 0;
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
  std::string log(static_cast<std::size_t>(std::max(0, length)), '\0');
  if (length > 0) {
    glGetShaderInfoLog(shader, length, nullptr, log.data());
  }
  std::cerr << "CAD shader compile error: " << log << "\n";
  glDeleteShader(shader);
  return 0;
}

bool CadModelRenderFeature::LinkShader(GlProgramHandle& program, const unsigned int vs, const unsigned int fs) {
  const unsigned int p = glCreateProgram();
  glAttachShader(p, vs);
  glAttachShader(p, fs);
  glLinkProgram(p);

  int ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (ok == GL_TRUE) {
    program.Set(p);
    return true;
  }

  int length = 0;
  glGetProgramiv(p, GL_INFO_LOG_LENGTH, &length);
  std::string log(static_cast<std::size_t>(std::max(0, length)), '\0');
  if (length > 0) {
    glGetProgramInfoLog(p, length, nullptr, log.data());
  }
  std::cerr << "CAD shader link error: " << log << "\n";
  glDeleteProgram(p);
  return false;
}

}  // namespace repulsor3d
