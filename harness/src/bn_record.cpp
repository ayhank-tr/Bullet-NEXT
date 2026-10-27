// Bullet NEXT golden harness - recorder and metrics (commits C0.6, C0.9).
#include "bn_record.h"

#include <cmath>
#include <cstring>

BN_NAMESPACE_BEGIN

Recorder::Recorder(const std::string& scene, World& w)
    : m_scene(scene), m_bodies((int)w.bodies().size()) {}

static void bodyState(const BodyRec& b, float* out) {
  btVector3 p(0, 0, 0), lv(0, 0, 0), av(0, 0, 0);
  btQuaternion q(0, 0, 0, 1);
  if (b.soft) {
    // Soft body: centre of its nodes; rotation unused; velocity = mean node velocity.
    const int n = b.soft->m_nodes.size();
    for (int i = 0; i < n; ++i) { p += b.soft->m_nodes[i].m_x; lv += b.soft->m_nodes[i].m_v; }
    if (n) { p /= btScalar(n); lv /= btScalar(n); }
  } else {
    const btTransform& t = b.obj->getWorldTransform();
    p = t.getOrigin();
    q = t.getRotation();
    if (b.rb) { lv = b.rb->getLinearVelocity(); av = b.rb->getAngularVelocity(); }
  }
  const float v[kFloatsPerBody] = {(float)p.x(), (float)p.y(), (float)p.z(),
                                   (float)q.x(), (float)q.y(), (float)q.z(), (float)q.w(),
                                   (float)lv.x(), (float)lv.y(), (float)lv.z(),
                                   (float)av.x(), (float)av.y(), (float)av.z()};
  std::memcpy(out, v, sizeof(v));
}

u64 hashFrame(World& w, std::vector<float>* state, std::vector<int>* evInts, std::vector<float>* evFloats) {
  Hash64 h;
  float s[kFloatsPerBody];
  const int bodies = (int)w.bodies().size();
  for (int i = 0; i < bodies; ++i) {
    const BodyRec& b = w.bodies()[i];
    bodyState(b, s);
    if (state) state->insert(state->end(), s, s + kFloatsPerBody);
    h.bytes(s, sizeof(s));
    if (b.soft) {  // every node position feeds the hash
      for (int n = 0; n < b.soft->m_nodes.size(); ++n) {
        const btVector3& x = b.soft->m_nodes[n].m_x;
        h.f((float)x.x()); h.f((float)x.y()); h.f((float)x.z());
      }
    }
  }
  const std::vector<ContactEvent>& ev = w.events();
  h.i((int)ev.size());
  for (size_t i = 0; i < ev.size(); ++i) {
    const ContactEvent& e = ev[i];
    const float f[9] = {(float)e.p1.x(), (float)e.p1.y(), (float)e.p1.z(),
                        (float)e.p2.x(), (float)e.p2.y(), (float)e.p2.z(),
                        (float)e.normal.x(), (float)e.normal.y(), (float)e.normal.z()};
    if (evInts) { evInts->push_back(e.a); evInts->push_back(e.b); }
    if (evFloats) evFloats->insert(evFloats->end(), f, f + 9);
    h.i(e.a); h.i(e.b); h.bytes(f, sizeof(f));
  }
  return h.h;
}

u64 chainHash(u64 chain, u64 frameHash) {
  Hash64 fin; fin.h = chain; fin.bytes(&frameHash, sizeof(frameHash));
  return fin.h;
}

void Recorder::capture(World& w, const FrameMetrics& m) {
  const u64 h = hashFrame(w, &m_state, &m_eventInts, &m_eventFloats);
  m_eventCounts.push_back((int)w.events().size());
  m_hashes.push_back(h);
  m_metrics.push_back(m);
  m_final = chainHash(m_final, h);
}

// File layout (little endian):
//   "BNR1" | u32 version | str backend | str scene | u32 bodies | u32 frames
//   per frame: f32[bodies*13] | u32 nEvents | (i32 a, i32 b, f32[9])* | u64 frameHash
//   u64 finalHash
static void putU32(FILE* f, std::uint32_t v) { fwrite(&v, 4, 1, f); }
static void putStr(FILE* f, const std::string& s) { putU32(f, (std::uint32_t)s.size()); fwrite(s.data(), 1, s.size(), f); }

bool Recorder::writeBinary(const std::string& path) const {
  FILE* f = fopen(path.c_str(), "wb");
  if (!f) return false;
  fwrite("BNR1", 1, 4, f);
  putU32(f, 1);
  putStr(f, BN_BACKEND_NAME);
  putStr(f, m_scene);
  putU32(f, (std::uint32_t)m_bodies);
  putU32(f, (std::uint32_t)m_hashes.size());
  size_t ev = 0;
  for (size_t fr = 0; fr < m_hashes.size(); ++fr) {
    fwrite(&m_state[fr * m_bodies * kFloatsPerBody], sizeof(float), (size_t)m_bodies * kFloatsPerBody, f);
    putU32(f, (std::uint32_t)m_eventCounts[fr]);
    for (int e = 0; e < m_eventCounts[fr]; ++e, ++ev) {
      fwrite(&m_eventInts[ev * 2], sizeof(int), 2, f);
      fwrite(&m_eventFloats[ev * 9], sizeof(float), 9, f);
    }
    fwrite(&m_hashes[fr], 8, 1, f);
  }
  fwrite(&m_final, 8, 1, f);
  fclose(f);
  return true;
}

bool Recorder::writeMetricsCsv(const std::string& path) const {
  FILE* f = fopen(path.c_str(), "w");
  if (!f) return false;
  fprintf(f, "frame,max_penetration_m,sum_penetration_m,energy_j,max_joint_drift_m,contacts,step_us,hash\n");
  for (size_t i = 0; i < m_metrics.size(); ++i) {
    const FrameMetrics& m = m_metrics[i];
    fprintf(f, "%zu,%.9g,%.9g,%.9g,%.9g,%d,%.3f,%016llx\n", i + 1, m.maxPenetration, m.sumPenetration,
            m.energy, m.maxJointDrift, m.contacts, m.stepMicros, (unsigned long long)m_hashes[i]);
  }
  fclose(f);
  return true;
}

static btVector3 pivotSeparation(btTypedConstraint* c) {
  const btTransform& ta = c->getRigidBodyA().getCenterOfMassTransform();
  const btTransform& tb = c->getRigidBodyB().getCenterOfMassTransform();
  switch (c->getUserConstraintType()) {
    case 1: {
      btPoint2PointConstraint* p = static_cast<btPoint2PointConstraint*>(c);
      return ta * p->getPivotInA() - tb * p->getPivotInB();
    }
    case 3:
      return btVector3(0, 0, 0);  // angular-only hinge: position is free by design
    case 2: {
      btHingeConstraint* h = static_cast<btHingeConstraint*>(c);
      return ta * h->getAFrame().getOrigin() - tb * h->getBFrame().getOrigin();
    }
    case 4: {
      btConeTwistConstraint* k = static_cast<btConeTwistConstraint*>(c);
      return ta * k->getAFrame().getOrigin() - tb * k->getBFrame().getOrigin();
    }
    default:
      return btVector3(0, 0, 0);  // 6DOF: translation is a degree of freedom, not an error
  }
}

FrameMetrics measure(World& w) {
  FrameMetrics m;
  btDispatcher* d = w.dyn()->getDispatcher();
  for (int i = 0; i < d->getNumManifolds(); ++i) {
    btPersistentManifold* pm = d->getManifoldByIndexInternal(i);
    for (int c = 0; c < pm->getNumContacts(); ++c) {
      const float dist = (float)pm->getContactPoint(c).getDistance();
      ++m.contacts;
      if (dist < 0) {
        m.sumPenetration += -dist;
        if (-dist > m.maxPenetration) m.maxPenetration = -dist;
      }
    }
  }
  const float g = -w.gravityZ();
  for (size_t i = 0; i < w.bodies().size(); ++i) {
    const BodyRec& b = w.bodies()[i];
    if (!b.rb || b.rb->getInvMass() == 0) continue;
    const btScalar mass = 1.0f / b.rb->getInvMass();
    const btVector3& v = b.rb->getLinearVelocity();
    const btVector3& av = b.rb->getAngularVelocity();
    const btVector3 inv = b.rb->getInvInertiaDiagLocal();
    const btMatrix3x3& R = b.rb->getCenterOfMassTransform().getBasis();
    const btVector3 wl = av * R;  // angular velocity in local frame
    btScalar rot = 0;
    for (int k = 0; k < 3; ++k) if (inv[k] != 0) rot += wl[k] * wl[k] / inv[k];
    m.energy += (float)(0.5f * mass * v.length2() + 0.5f * rot +
                        mass * g * b.rb->getCenterOfMassPosition().z());
  }
  for (size_t i = 0; i < w.constraints().size(); ++i) {
    const float drift = (float)pivotSeparation(w.constraints()[i]).length();
    if (drift > m.maxJointDrift) m.maxJointDrift = drift;
  }
  return m;
}

BN_NAMESPACE_END  // namespace bn
