Import("env")

board_config = env.BoardConfig()
# should be array of VID:PID pairs
board_config.update("build.hwids", [
  ["0x256f", "0xc631"],  # SpaceMouse PID & VID
])