from gym.envs.registration import register

register(
        id='HopperEnv-v0',
        entry_point='MyGym.Envs.HopperEnv:HopperBulletEnv',
        max_episode_steps=1000,
        reward_threshold=2500.0
        )

register(
        id='AtlasEnv-v0',
        entry_point='MyGym.Envs.AtlasEnv:AtlasBulletEnv',
        max_episode_steps=1000
        )

register(
        id='HumanoidEnv-v0',
        entry_point='MyGym.Envs.HumanoidEnv:HumanoidBulletEnv',
        max_episode_steps=1000
        )

register(
        id='DracoEnv-v0',
        entry_point='MyGym.Envs.DracoEnv:DracoBulletEnv',
        max_episode_steps=1000
        )

register(
        id='DummyDracoEnv-v0',
        entry_point='MyGym.Envs.DummyDracoEnv:DummyDracoEnv',
        max_episode_steps=1000
        )

register(
        id='DummyAtlasEnv-v0',
        entry_point='MyGym.Envs.DummyAtlasEnv:DummyAtlasEnv',
        max_episode_steps=1000
        )

register(
        id='DummyCartPoleEnv-v0',
        entry_point='MyGym.Envs.DummyCartPoleEnv:DummyCartPoleEnv',
        max_episode_steps=1000
        )
