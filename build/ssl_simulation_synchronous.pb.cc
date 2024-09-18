// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: ssl_simulation_synchronous.proto

#include "ssl_simulation_synchronous.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
extern PROTOBUF_INTERNAL_EXPORT_ssl_5fsimulation_5frobot_5fcontrol_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_RobotControl_ssl_5fsimulation_5frobot_5fcontrol_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_ssl_5fsimulation_5frobot_5ffeedback_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_RobotControlResponse_ssl_5fsimulation_5frobot_5ffeedback_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_ssl_5fvision_5fdetection_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_SSL_DetectionFrame_ssl_5fvision_5fdetection_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_ssl_5fsimulation_5fcontrol_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_SimulatorCommand_ssl_5fsimulation_5fcontrol_2eproto;
class SimulationSyncRequestDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SimulationSyncRequest> _instance;
} _SimulationSyncRequest_default_instance_;
class SimulationSyncResponseDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<SimulationSyncResponse> _instance;
} _SimulationSyncResponse_default_instance_;
static void InitDefaultsscc_info_SimulationSyncRequest_ssl_5fsimulation_5fsynchronous_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::_SimulationSyncRequest_default_instance_;
    new (ptr) ::SimulationSyncRequest();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::SimulationSyncRequest::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_SimulationSyncRequest_ssl_5fsimulation_5fsynchronous_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_SimulationSyncRequest_ssl_5fsimulation_5fsynchronous_2eproto}, {
      &scc_info_SimulatorCommand_ssl_5fsimulation_5fcontrol_2eproto.base,
      &scc_info_RobotControl_ssl_5fsimulation_5frobot_5fcontrol_2eproto.base,}};

static void InitDefaultsscc_info_SimulationSyncResponse_ssl_5fsimulation_5fsynchronous_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::_SimulationSyncResponse_default_instance_;
    new (ptr) ::SimulationSyncResponse();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::SimulationSyncResponse::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_SimulationSyncResponse_ssl_5fsimulation_5fsynchronous_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, 0, InitDefaultsscc_info_SimulationSyncResponse_ssl_5fsimulation_5fsynchronous_2eproto}, {
      &scc_info_SSL_DetectionFrame_ssl_5fvision_5fdetection_2eproto.base,
      &scc_info_RobotControlResponse_ssl_5fsimulation_5frobot_5ffeedback_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_ssl_5fsimulation_5fsynchronous_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_ssl_5fsimulation_5fsynchronous_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_ssl_5fsimulation_5fsynchronous_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_ssl_5fsimulation_5fsynchronous_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::SimulationSyncRequest, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::SimulationSyncRequest, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::SimulationSyncRequest, sim_step_),
  PROTOBUF_FIELD_OFFSET(::SimulationSyncRequest, simulator_command_),
  PROTOBUF_FIELD_OFFSET(::SimulationSyncRequest, robot_control_),
  2,
  0,
  1,
  PROTOBUF_FIELD_OFFSET(::SimulationSyncResponse, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::SimulationSyncResponse, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::SimulationSyncResponse, detection_),
  PROTOBUF_FIELD_OFFSET(::SimulationSyncResponse, robot_control_response_),
  ~0u,
  0,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::SimulationSyncRequest)},
  { 11, 18, sizeof(::SimulationSyncResponse)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::_SimulationSyncRequest_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::_SimulationSyncResponse_default_instance_),
};

const char descriptor_table_protodef_ssl_5fsimulation_5fsynchronous_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n ssl_simulation_synchronous.proto\032\032ssl_"
  "vision_detection.proto\032#ssl_simulation_r"
  "obot_feedback.proto\032\"ssl_simulation_robo"
  "t_control.proto\032\034ssl_simulation_control."
  "proto\"}\n\025SimulationSyncRequest\022\020\n\010sim_st"
  "ep\030\001 \001(\002\022,\n\021simulator_command\030\002 \001(\0132\021.Si"
  "mulatorCommand\022$\n\rrobot_control\030\003 \001(\0132\r."
  "RobotControl\"w\n\026SimulationSyncResponse\022&"
  "\n\tdetection\030\001 \003(\0132\023.SSL_DetectionFrame\0225"
  "\n\026robot_control_response\030\002 \001(\0132\025.RobotCo"
  "ntrolResponseB8Z6github.com/RoboCup-SSL/"
  "ssl-simulation-protocol/pkg/sim"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_ssl_5fsimulation_5fsynchronous_2eproto_deps[4] = {
  &::descriptor_table_ssl_5fsimulation_5fcontrol_2eproto,
  &::descriptor_table_ssl_5fsimulation_5frobot_5fcontrol_2eproto,
  &::descriptor_table_ssl_5fsimulation_5frobot_5ffeedback_2eproto,
  &::descriptor_table_ssl_5fvision_5fdetection_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_ssl_5fsimulation_5fsynchronous_2eproto_sccs[2] = {
  &scc_info_SimulationSyncRequest_ssl_5fsimulation_5fsynchronous_2eproto.base,
  &scc_info_SimulationSyncResponse_ssl_5fsimulation_5fsynchronous_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_ssl_5fsimulation_5fsynchronous_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_ssl_5fsimulation_5fsynchronous_2eproto = {
  false, false, descriptor_table_protodef_ssl_5fsimulation_5fsynchronous_2eproto, "ssl_simulation_synchronous.proto", 471,
  &descriptor_table_ssl_5fsimulation_5fsynchronous_2eproto_once, descriptor_table_ssl_5fsimulation_5fsynchronous_2eproto_sccs, descriptor_table_ssl_5fsimulation_5fsynchronous_2eproto_deps, 2, 4,
  schemas, file_default_instances, TableStruct_ssl_5fsimulation_5fsynchronous_2eproto::offsets,
  file_level_metadata_ssl_5fsimulation_5fsynchronous_2eproto, 2, file_level_enum_descriptors_ssl_5fsimulation_5fsynchronous_2eproto, file_level_service_descriptors_ssl_5fsimulation_5fsynchronous_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_ssl_5fsimulation_5fsynchronous_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_ssl_5fsimulation_5fsynchronous_2eproto)), true);

// ===================================================================

void SimulationSyncRequest::InitAsDefaultInstance() {
  ::_SimulationSyncRequest_default_instance_._instance.get_mutable()->simulator_command_ = const_cast< ::SimulatorCommand*>(
      ::SimulatorCommand::internal_default_instance());
  ::_SimulationSyncRequest_default_instance_._instance.get_mutable()->robot_control_ = const_cast< ::RobotControl*>(
      ::RobotControl::internal_default_instance());
}
class SimulationSyncRequest::_Internal {
 public:
  using HasBits = decltype(std::declval<SimulationSyncRequest>()._has_bits_);
  static void set_has_sim_step(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static const ::SimulatorCommand& simulator_command(const SimulationSyncRequest* msg);
  static void set_has_simulator_command(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::RobotControl& robot_control(const SimulationSyncRequest* msg);
  static void set_has_robot_control(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
};

const ::SimulatorCommand&
SimulationSyncRequest::_Internal::simulator_command(const SimulationSyncRequest* msg) {
  return *msg->simulator_command_;
}
const ::RobotControl&
SimulationSyncRequest::_Internal::robot_control(const SimulationSyncRequest* msg) {
  return *msg->robot_control_;
}
void SimulationSyncRequest::clear_simulator_command() {
  if (simulator_command_ != nullptr) simulator_command_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void SimulationSyncRequest::clear_robot_control() {
  if (robot_control_ != nullptr) robot_control_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
SimulationSyncRequest::SimulationSyncRequest(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:SimulationSyncRequest)
}
SimulationSyncRequest::SimulationSyncRequest(const SimulationSyncRequest& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_simulator_command()) {
    simulator_command_ = new ::SimulatorCommand(*from.simulator_command_);
  } else {
    simulator_command_ = nullptr;
  }
  if (from._internal_has_robot_control()) {
    robot_control_ = new ::RobotControl(*from.robot_control_);
  } else {
    robot_control_ = nullptr;
  }
  sim_step_ = from.sim_step_;
  // @@protoc_insertion_point(copy_constructor:SimulationSyncRequest)
}

void SimulationSyncRequest::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_SimulationSyncRequest_ssl_5fsimulation_5fsynchronous_2eproto.base);
  ::memset(&simulator_command_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&sim_step_) -
      reinterpret_cast<char*>(&simulator_command_)) + sizeof(sim_step_));
}

SimulationSyncRequest::~SimulationSyncRequest() {
  // @@protoc_insertion_point(destructor:SimulationSyncRequest)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void SimulationSyncRequest::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  if (this != internal_default_instance()) delete simulator_command_;
  if (this != internal_default_instance()) delete robot_control_;
}

void SimulationSyncRequest::ArenaDtor(void* object) {
  SimulationSyncRequest* _this = reinterpret_cast< SimulationSyncRequest* >(object);
  (void)_this;
}
void SimulationSyncRequest::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void SimulationSyncRequest::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SimulationSyncRequest& SimulationSyncRequest::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SimulationSyncRequest_ssl_5fsimulation_5fsynchronous_2eproto.base);
  return *internal_default_instance();
}


void SimulationSyncRequest::Clear() {
// @@protoc_insertion_point(message_clear_start:SimulationSyncRequest)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(simulator_command_ != nullptr);
      simulator_command_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(robot_control_ != nullptr);
      robot_control_->Clear();
    }
  }
  sim_step_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SimulationSyncRequest::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional float sim_step = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 13)) {
          _Internal::set_has_sim_step(&has_bits);
          sim_step_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<float>(ptr);
          ptr += sizeof(float);
        } else goto handle_unusual;
        continue;
      // optional .SimulatorCommand simulator_command = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_simulator_command(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .RobotControl robot_control = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_robot_control(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* SimulationSyncRequest::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:SimulationSyncRequest)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional float sim_step = 1;
  if (cached_has_bits & 0x00000004u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteFloatToArray(1, this->_internal_sim_step(), target);
  }

  // optional .SimulatorCommand simulator_command = 2;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::simulator_command(this), target, stream);
  }

  // optional .RobotControl robot_control = 3;
  if (cached_has_bits & 0x00000002u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        3, _Internal::robot_control(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:SimulationSyncRequest)
  return target;
}

size_t SimulationSyncRequest::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:SimulationSyncRequest)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional .SimulatorCommand simulator_command = 2;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *simulator_command_);
    }

    // optional .RobotControl robot_control = 3;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *robot_control_);
    }

    // optional float sim_step = 1;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 + 4;
    }

  }
  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void SimulationSyncRequest::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:SimulationSyncRequest)
  GOOGLE_DCHECK_NE(&from, this);
  const SimulationSyncRequest* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SimulationSyncRequest>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:SimulationSyncRequest)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:SimulationSyncRequest)
    MergeFrom(*source);
  }
}

void SimulationSyncRequest::MergeFrom(const SimulationSyncRequest& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:SimulationSyncRequest)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _internal_mutable_simulator_command()->::SimulatorCommand::MergeFrom(from._internal_simulator_command());
    }
    if (cached_has_bits & 0x00000002u) {
      _internal_mutable_robot_control()->::RobotControl::MergeFrom(from._internal_robot_control());
    }
    if (cached_has_bits & 0x00000004u) {
      sim_step_ = from.sim_step_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void SimulationSyncRequest::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:SimulationSyncRequest)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SimulationSyncRequest::CopyFrom(const SimulationSyncRequest& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:SimulationSyncRequest)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SimulationSyncRequest::IsInitialized() const {
  if (_internal_has_simulator_command()) {
    if (!simulator_command_->IsInitialized()) return false;
  }
  if (_internal_has_robot_control()) {
    if (!robot_control_->IsInitialized()) return false;
  }
  return true;
}

void SimulationSyncRequest::InternalSwap(SimulationSyncRequest* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(SimulationSyncRequest, sim_step_)
      + sizeof(SimulationSyncRequest::sim_step_)
      - PROTOBUF_FIELD_OFFSET(SimulationSyncRequest, simulator_command_)>(
          reinterpret_cast<char*>(&simulator_command_),
          reinterpret_cast<char*>(&other->simulator_command_));
}

::PROTOBUF_NAMESPACE_ID::Metadata SimulationSyncRequest::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void SimulationSyncResponse::InitAsDefaultInstance() {
  ::_SimulationSyncResponse_default_instance_._instance.get_mutable()->robot_control_response_ = const_cast< ::RobotControlResponse*>(
      ::RobotControlResponse::internal_default_instance());
}
class SimulationSyncResponse::_Internal {
 public:
  using HasBits = decltype(std::declval<SimulationSyncResponse>()._has_bits_);
  static const ::RobotControlResponse& robot_control_response(const SimulationSyncResponse* msg);
  static void set_has_robot_control_response(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

const ::RobotControlResponse&
SimulationSyncResponse::_Internal::robot_control_response(const SimulationSyncResponse* msg) {
  return *msg->robot_control_response_;
}
void SimulationSyncResponse::clear_detection() {
  detection_.Clear();
}
void SimulationSyncResponse::clear_robot_control_response() {
  if (robot_control_response_ != nullptr) robot_control_response_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
SimulationSyncResponse::SimulationSyncResponse(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena),
  detection_(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:SimulationSyncResponse)
}
SimulationSyncResponse::SimulationSyncResponse(const SimulationSyncResponse& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      detection_(from.detection_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_robot_control_response()) {
    robot_control_response_ = new ::RobotControlResponse(*from.robot_control_response_);
  } else {
    robot_control_response_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:SimulationSyncResponse)
}

void SimulationSyncResponse::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_SimulationSyncResponse_ssl_5fsimulation_5fsynchronous_2eproto.base);
  robot_control_response_ = nullptr;
}

SimulationSyncResponse::~SimulationSyncResponse() {
  // @@protoc_insertion_point(destructor:SimulationSyncResponse)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void SimulationSyncResponse::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  if (this != internal_default_instance()) delete robot_control_response_;
}

void SimulationSyncResponse::ArenaDtor(void* object) {
  SimulationSyncResponse* _this = reinterpret_cast< SimulationSyncResponse* >(object);
  (void)_this;
}
void SimulationSyncResponse::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void SimulationSyncResponse::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const SimulationSyncResponse& SimulationSyncResponse::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_SimulationSyncResponse_ssl_5fsimulation_5fsynchronous_2eproto.base);
  return *internal_default_instance();
}


void SimulationSyncResponse::Clear() {
// @@protoc_insertion_point(message_clear_start:SimulationSyncResponse)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  detection_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(robot_control_response_ != nullptr);
    robot_control_response_->Clear();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* SimulationSyncResponse::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .SSL_DetectionFrame detection = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_detection(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<10>(ptr));
        } else goto handle_unusual;
        continue;
      // optional .RobotControlResponse robot_control_response = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_robot_control_response(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* SimulationSyncResponse::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:SimulationSyncResponse)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .SSL_DetectionFrame detection = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_detection_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, this->_internal_detection(i), target, stream);
  }

  cached_has_bits = _has_bits_[0];
  // optional .RobotControlResponse robot_control_response = 2;
  if (cached_has_bits & 0x00000001u) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(
        2, _Internal::robot_control_response(this), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:SimulationSyncResponse)
  return target;
}

size_t SimulationSyncResponse::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:SimulationSyncResponse)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .SSL_DetectionFrame detection = 1;
  total_size += 1UL * this->_internal_detection_size();
  for (const auto& msg : this->detection_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // optional .RobotControlResponse robot_control_response = 2;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *robot_control_response_);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void SimulationSyncResponse::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:SimulationSyncResponse)
  GOOGLE_DCHECK_NE(&from, this);
  const SimulationSyncResponse* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<SimulationSyncResponse>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:SimulationSyncResponse)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:SimulationSyncResponse)
    MergeFrom(*source);
  }
}

void SimulationSyncResponse::MergeFrom(const SimulationSyncResponse& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:SimulationSyncResponse)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  detection_.MergeFrom(from.detection_);
  if (from._internal_has_robot_control_response()) {
    _internal_mutable_robot_control_response()->::RobotControlResponse::MergeFrom(from._internal_robot_control_response());
  }
}

void SimulationSyncResponse::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:SimulationSyncResponse)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SimulationSyncResponse::CopyFrom(const SimulationSyncResponse& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:SimulationSyncResponse)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SimulationSyncResponse::IsInitialized() const {
  if (!::PROTOBUF_NAMESPACE_ID::internal::AllAreInitialized(detection_)) return false;
  if (_internal_has_robot_control_response()) {
    if (!robot_control_response_->IsInitialized()) return false;
  }
  return true;
}

void SimulationSyncResponse::InternalSwap(SimulationSyncResponse* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  detection_.InternalSwap(&other->detection_);
  swap(robot_control_response_, other->robot_control_response_);
}

::PROTOBUF_NAMESPACE_ID::Metadata SimulationSyncResponse::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::SimulationSyncRequest* Arena::CreateMaybeMessage< ::SimulationSyncRequest >(Arena* arena) {
  return Arena::CreateMessageInternal< ::SimulationSyncRequest >(arena);
}
template<> PROTOBUF_NOINLINE ::SimulationSyncResponse* Arena::CreateMaybeMessage< ::SimulationSyncResponse >(Arena* arena) {
  return Arena::CreateMessageInternal< ::SimulationSyncResponse >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
