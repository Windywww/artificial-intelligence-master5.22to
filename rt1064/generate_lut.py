# generate_lut.py
import sys

def get_required_tnt(env_mask, is_bomb):
    """
    解析 16-bit 掩码，并跑一遍我们之前写好的终极死锁判定逻辑。
    返回打破该死锁需要的最小 TNT 数量 (0: 安全, 1: 需1颗, 2: 需2颗, 255: 绝对死局)
    """
    # 解析 8 个格子的状态 (0:空, 1:内墙, 2:box或边界墙  3：tnt)
    c_TL = (env_mask >> 0) & 0b11
    c_T  = (env_mask >> 2) & 0b11
    c_TR = (env_mask >> 4) & 0b11
    c_L  = (env_mask >> 6) & 0b11
    c_R  = (env_mask >> 8) & 0b11
    c_BL = (env_mask >> 10) & 0b11
    c_B  = (env_mask >> 12) & 0b11
    c_BR = (env_mask >> 14) & 0b11

    # 闭包辅助函数 (对接上一轮的逻辑)
    def is_unbreakable(val): return val == 2 or val == 3
    def is_solid(val):      return val == 1 or val == 2 or val == 3
    def is_tnt(val):       return val == 3
    def is_any_wall(val):    return val == 1 or val == 2  # 把box也当墙
    def is_bb(val):          return val == 2
    def is_innerwall(val):      return val == 1

    # 1. 绝对 2x2 死锁 (全不可破坏) -> 需要 255 颗炸弹 (神仙难救)
    if is_unbreakable(c_R) and is_unbreakable(c_B) and is_unbreakable(c_BR): return 255
    if is_unbreakable(c_L) and is_unbreakable(c_B) and is_unbreakable(c_BL): return 255
    if is_unbreakable(c_R) and is_unbreakable(c_T) and is_unbreakable(c_TR): return 255
    if is_unbreakable(c_L) and is_unbreakable(c_T) and is_unbreakable(c_TL): return 255

    # 2. 墙角与口袋死锁
    # 2.1 U形口袋死锁  return很准确，严格分类讨论过，勿动！！！
    if is_unbreakable(c_R) and is_solid(c_B) and is_solid(c_BR) and is_solid(c_T) and is_solid(c_TR): return 2 if is_bomb else 1
    if is_unbreakable(c_L) and is_solid(c_B) and is_solid(c_BL) and is_solid(c_T) and is_solid(c_TL): return 2 if is_bomb else 1
    if is_unbreakable(c_B) and is_solid(c_R) and is_solid(c_BR) and is_solid(c_L) and is_solid(c_BL): return 2 if is_bomb else 1
    if is_unbreakable(c_T) and is_solid(c_R) and is_solid(c_TR) and is_solid(c_L) and is_solid(c_TL): return 2 if is_bomb else 1

    # 2.2 软性 2x2 死锁
    if is_solid(c_R) and is_solid(c_B) and is_solid(c_BR): 
        if is_bb(c_R) and is_bb(c_B): return 2 if is_bomb else 1
        if is_innerwall(c_R) or is_innerwall(c_B): return 1
    if is_solid(c_L) and is_solid(c_B) and is_solid(c_BL):
        if is_bb(c_L) and is_bb(c_B): return 2 if is_bomb else 1
        if is_innerwall(c_L) or is_innerwall(c_B): return 1
    if is_solid(c_R) and is_solid(c_T) and is_solid(c_TR): 
        if is_bb(c_R) and is_bb(c_T): return 2 if is_bomb else 1
        if is_innerwall(c_R) or is_innerwall(c_T): return 1      
    if is_solid(c_L) and is_solid(c_T) and is_solid(c_TL):
        if is_bb(c_L) and is_bb(c_T): return 2 if is_bomb else 1       
        if is_innerwall(c_L) or is_innerwall(c_T): return 1

    # 2.3 L型墙角
    def corner_deadlock(d1, d2):
        if is_any_wall(d1) and is_any_wall(d2):
            if is_unbreakable(d1) and is_unbreakable(d2): return 255 # 神仙难救
            return 1
        return 0

    res = max(corner_deadlock(c_T, c_L), corner_deadlock(c_T, c_R), 
              corner_deadlock(c_B, c_L), corner_deadlock(c_B, c_R))
    return res


def generate_c_file():
    with open("sokoban_lut.c", "w") as f:
        f.write("#include \"sokoban_lut.h\"\n\n")
        
        # 生成 BOX 的查表
        f.write("// Box Deadlock Table (Value represents minimum TNT required to break, 255 = absolute deadlock)\n")
        f.write("const uint8_t DEADLOCK_LUT_BOX[65536] = {\n    ")
        for mask in range(65536):
            val = get_required_tnt(mask, is_bomb=False)
            f.write(f"{val}, ")
            if (mask + 1) % 32 == 0:
                f.write("\n    ")
        f.write("};\n\n")

        # 生成 BOMB 的查表
        f.write("// Bomb Deadlock Table\n")
        f.write("const uint8_t DEADLOCK_LUT_BOMB[65536] = {\n    ")
        for mask in range(65536):
            val = get_required_tnt(mask, is_bomb=True)
            f.write(f"{val}, ")
            if (mask + 1) % 32 == 0:
                f.write("\n    ")
        f.write("};\n")

if __name__ == "__main__":
    print("Generating sokoban_lut.c (128 KB)...")
    generate_c_file()
    print("Done! You can now compile this file into your RT1064 project.")