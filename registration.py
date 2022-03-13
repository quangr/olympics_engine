from envpool.registration import register

register(
  task_id="Curling-v0",
  import_path="envpool.classic_control",
  spec_cls="CurlingEnvSpec",
  dm_cls="CurlingDMEnvPool",
  gym_cls="CurlingGymEnvPool",
  max_episode_steps=200,
  reward_threshold=195.0,
)

register(
  task_id="Curling-v1",
  import_path="envpool.classic_control",
  spec_cls="CurlingEnvSpec",
  dm_cls="CurlingDMEnvPool",
  gym_cls="CurlingGymEnvPool",
  max_episode_steps=500,
  reward_threshold=475.0,
)