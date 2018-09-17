from gym.envs.registration import registry, register, make, spec

register(
        id='BaxterReacherEnv-v0',
        entry_point='ripl_control.envs.baxter_envs.baxter_reacher_env:BaxterReacherEnv',
        # kwargs=kwargs,
        max_episode_steps=50,
)
