#pragma once

#include <GL/glew.h>

namespace repulsor3d {

class GlProgramHandle {
 public:
  GlProgramHandle() = default;
  ~GlProgramHandle() { Reset(); }

  GlProgramHandle(const GlProgramHandle&) = delete;
  GlProgramHandle& operator=(const GlProgramHandle&) = delete;

  GlProgramHandle(GlProgramHandle&& other) noexcept : id_(other.id_) { other.id_ = 0; }
  GlProgramHandle& operator=(GlProgramHandle&& other) noexcept {
    if (this != &other) {
      Reset();
      id_ = other.id_;
      other.id_ = 0;
    }
    return *this;
  }

  void Set(GLuint id) {
    Reset();
    id_ = id;
  }
  void Reset() {
    if (id_ != 0) {
      glDeleteProgram(id_);
      id_ = 0;
    }
  }
  GLuint Get() const { return id_; }

 private:
  GLuint id_ = 0;
};

class GlBufferHandle {
 public:
  GlBufferHandle() = default;
  ~GlBufferHandle() { Reset(); }

  GlBufferHandle(const GlBufferHandle&) = delete;
  GlBufferHandle& operator=(const GlBufferHandle&) = delete;

  GlBufferHandle(GlBufferHandle&& other) noexcept : id_(other.id_) { other.id_ = 0; }
  GlBufferHandle& operator=(GlBufferHandle&& other) noexcept {
    if (this != &other) {
      Reset();
      id_ = other.id_;
      other.id_ = 0;
    }
    return *this;
  }

  void Set(GLuint id) {
    Reset();
    id_ = id;
  }
  void Reset() {
    if (id_ != 0) {
      glDeleteBuffers(1, &id_);
      id_ = 0;
    }
  }
  GLuint Get() const { return id_; }

 private:
  GLuint id_ = 0;
};

class GlVertexArrayHandle {
 public:
  GlVertexArrayHandle() = default;
  ~GlVertexArrayHandle() { Reset(); }

  GlVertexArrayHandle(const GlVertexArrayHandle&) = delete;
  GlVertexArrayHandle& operator=(const GlVertexArrayHandle&) = delete;

  GlVertexArrayHandle(GlVertexArrayHandle&& other) noexcept : id_(other.id_) { other.id_ = 0; }
  GlVertexArrayHandle& operator=(GlVertexArrayHandle&& other) noexcept {
    if (this != &other) {
      Reset();
      id_ = other.id_;
      other.id_ = 0;
    }
    return *this;
  }

  void Set(GLuint id) {
    Reset();
    id_ = id;
  }
  void Reset() {
    if (id_ != 0) {
      glDeleteVertexArrays(1, &id_);
      id_ = 0;
    }
  }
  GLuint Get() const { return id_; }

 private:
  GLuint id_ = 0;
};

}  // namespace repulsor3d
