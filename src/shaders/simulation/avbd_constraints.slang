#pragma once
#include "avbd_common.slang"

daxa_f32vec3 avbd_small_angle_delta(Quaternion q_now, Quaternion q_start)
{
  Quaternion qs_conj = Quaternion(-q_start.v, q_start.w);
  Quaternion dq = q_now * qs_conj;
  daxa_f32 sign = dq.w < 0.0f ? -1.0f : 1.0f;
  return dq.v * (2.0f * sign);
}

// The current pose delta is shared by all contacts of one manifold visit.
// Recompute on every visit: another color may have updated the partner since
// the previous sweep. This is thread-local reuse, not a cross-sweep pose cache.
struct AvbdMotion
{
  daxa_f32vec3 translation;
  daxa_f32vec3 rotation;
};

AvbdMotion avbd_motion(RigidBody* r, AvbdBodyState* s, bool dynamic)
{
  AvbdMotion motion;
  motion.translation = daxa_f32vec3(0.0f);
  motion.rotation = daxa_f32vec3(0.0f);
  if (dynamic)
  {
    motion.translation = r->position - s->pos_start;
    motion.rotation = avbd_small_angle_delta(r->rotation, s->rot_start);
  }
  return motion;
}

daxa_f32 avbd_contact_C(Manifold* m, daxa_u32 j, RigidBody* r1, RigidBody* r2,
                        AvbdBodyState* s1, AvbdBodyState* s2,
                        bool dyn1, bool dyn2, daxa_f32 stab_alpha, AvbdMotion motion1, AvbdMotion motion2,
                        out daxa_f32vec3 rr1, out daxa_f32vec3 rr2)
{
  daxa_f32vec3 n = m->normal;
  daxa_f32vec3 p = m->contacts[j].position;
  // world arms relative to the step-start centers (statics: current center == start center)
  daxa_f32vec3 c1 = dyn1 ? s1->pos_start : r1->position;
  daxa_f32vec3 c2 = dyn2 ? s2->pos_start : r2->position;
  rr1 = p - c1;
  rr2 = p - c2;

  daxa_f32 C0 = m->contacts[j].penetration + BB_AVBD_MARGIN;
  // post-stab slop (normal-row sibling of BB_AVBD_STICK_SLOP, same measured rationale):
  // sub-slop penetration gets NO positional correction - correcting it every frame is the
  // push-out/re-settle limit cycle visible as rest jitter. Main sweeps (stab_alpha=1) hide
  // C0 entirely, so this only shapes the positional pass; depth beyond the slop corrects.
  if (stab_alpha < 0.5f && C0 < 0.0f) { C0 = min(C0 + BB_AVBD_PS_SLOP, 0.0f); }
  daxa_f32 C = C0 * (1.0f - stab_alpha);
  if (dyn1)
  {
    C -= dot(n, motion1.translation + cross(motion1.rotation, rr1));
  }
  if (dyn2)
  {
    C += dot(n, motion2.translation + cross(motion2.rotation, rr2));
  }
  return C;
}

daxa_f32vec3 avbd_q_rotate(Quaternion q, daxa_f32vec3 u)
{
  return u + 2.0f * cross(q.v, cross(q.v, u) + q.w * u);
}

daxa_f32 avbd_dir_C(daxa_f32vec3 d, Manifold* m, daxa_u32 j,
                    RigidBody* r1, RigidBody* r2,
                    AvbdBodyState* s1, AvbdBodyState* s2,
                    bool dyn1, bool dyn2, daxa_f32 stab_alpha, AvbdMotion motion1, AvbdMotion motion2,
                    out daxa_f32vec3 ar1, out daxa_f32vec3 ar2, out daxa_f32 C0t)
{
  // step-start poses (statics: current pose == start pose)
  daxa_f32vec3 c1 = dyn1 ? s1->pos_start : r1->position;
  daxa_f32vec3 c2 = dyn2 ? s2->pos_start : r2->position;
  Quaternion q1 = dyn1 ? Quaternion(s1->rot_start.v, s1->rot_start.w) : Quaternion(r1->rotation.v, r1->rotation.w);
  Quaternion q2 = dyn2 ? Quaternion(s2->rot_start.v, s2->rot_start.w) : Quaternion(r2->rotation.v, r2->rotation.w);
  // world arms of the anchored contact points at step start
  ar1 = avbd_q_rotate(q1, m->contacts[j].anchor_a);
  ar2 = avbd_q_rotate(q2, m->contacts[j].anchor_b);
  // pre-existing tangential drift between the anchors (C grows when body2 leads along +d).
  // The positional pull-back only acts on the drift beyond a small slop: a deadband that
  // keeps the post-stabilization sweep from perpetually micro-correcting compressed pile
  // contacts (sub-mm jitter that vetoes sleeping). Drift stays bounded by the slop.
  C0t = dot(d, (c2 + ar2) - (c1 + ar1));
  daxa_f32 c0_eff = sign(C0t) * max(0.0f, abs(C0t) - BB_AVBD_STICK_SLOP);
  daxa_f32 C = c0_eff * (1.0f - stab_alpha);
  if (dyn1)
  {
    C -= dot(d, motion1.translation + cross(motion1.rotation, ar1));
  }
  if (dyn2)
  {
    C += dot(d, motion2.translation + cross(motion2.rotation, ar2));
  }
  return C;
}
