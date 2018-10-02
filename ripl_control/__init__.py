from gym.envs.registration import registry, register, make, spec

register(
        id='BaxterReacherEnv-v0',
        entry_point='ripl_control.envs.baxter_envs.reach:BaxterReacherEnv',
        kwargs={'sim': False},
        max_episode_steps=200,
)

register(
        id='BaxterReacherEnv-v1',
        entry_point='ripl_control.envs.baxter_envs.reach:BaxterReacherEnv',
        kwargs={'sim': True},
        max_episode_steps=200,
)

register(
        id='BaxterPusherEnv-v0',
        entry_point='ripl_control.envs.baxter_envs.push:BaxterPusherEnv',
        # kwargs=kwargs,
        max_episode_steps=50,
)

register(
        id='BaxterSliderEnv-v0',
        entry_point='ripl_control.envs.baxter_envs.slide:BaxterSliderEnv',
        # kwargs=kwargs,
        max_episode_steps=50,
)

register(
        id='BaxterPickPlaceEnv-v0',
        entry_point='ripl_control.envs.baxter_envs.pick_and_place:BaxterPickPlaceEnv',
        # kwargs=kwargs,
        max_episode_steps=50,
)
