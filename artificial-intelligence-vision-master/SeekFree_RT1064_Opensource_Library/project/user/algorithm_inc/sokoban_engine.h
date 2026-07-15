#ifndef SOKOBAN_ENGINE_H
#define SOKOBAN_ENGINE_H

#include <stdint.h>
#include <stdbool.h>

// ========================================
// 1. �������ڴ�궨��
// ==========================================
#define WIDTH 16
#define HEIGHT 12
#define MAP_SIZE (WIDTH * HEIGHT)
#define MAX_BOXES 6
#define MAX_GOALS 6 // ����state����uint16_t��λͼ����¼Ŀ��״̬�����֧��16��Ŀ��!!!
#define MAX_BOMBS 4

#define VIRTUAL_WALL_COST 20
#define BOMB_PENALTY 10 // ���壺�����ǵ���ը����ʱ����൱��car��2��g += 2��  ��ʵ��0���ã�
#define RES_SUCCESS -1.0f
#define RES_INF 9999.0f
#define MAX_STEPS 100 // ����������ڳ�ʼ��action����
#define INF_DIST 9999
#define NO_CLS 0 // �޷���ģʽ����Ĭ��id

#define HASH_TABLE_SIZE 1048576 // 1MB *16(HashEntry) = 16MB
#define HASH_MASK (HASH_TABLE_SIZE - 1)
// 【新增】最大分支数 (单层展开的子节点上限)
// �?6个箱�?4个炸�? = 10*4 = 40
#define MAX_BRANCHES 28

// ==========================================
// ״̬���ݽṹ (State)
// ���ý����ͽṹ�壬������� Zobrist Hashing �� Memcmp
// ����������ʹ��
typedef struct
{
    uint8_t pos;
    uint8_t id;
} EntityData;

typedef struct
{
    uint8_t car_pos;
    uint8_t box_count;
    EntityData boxes[MAX_BOXES];
    uint8_t bomb_count;
    uint8_t bombs[MAX_BOMBS];
    // uint8_t  walls[MAP_SIZE];  ������
    //  Ŀ�����λͼ���� i λΪ 1 ��ʾ ctx->goals[i] ��δ������
    uint16_t active_goals_mask;
    uint64_t base_hash; // Zobrist ������ϣ��ֵ
} State;                // 32 bytes

// �궯����¼
typedef struct
{
    uint8_t move_to; // С��������
    uint8_t push_to; // ������ŵ�
    bool is_explode; // �Ƿ����ը����ը
    bool is_consume; // �Ƿ�����Ŀ��
} MacroAction;

// �켣���ݽṹ (���� M ��ģ�������)
typedef struct
{
    uint8_t points[MAP_SIZE]; // ·�������� (���������յ�)
    uint16_t length;          // ·���ܳ���
} WaypointPath;
// ==========================================
// �������������� (SokobanContext)
typedef struct
{
    uint8_t boundary_walls[MAP_SIZE];
    State initial_state; // ��¼��һ��ִ��generate_path���״̬��
    int8_t dir_offsets[4];
    uint8_t explosion_areas[MAP_SIZE][9];   // �洢ÿ���㱬ը�ܲ����ĸ�������
    uint8_t explosion_area_count[MAP_SIZE]; // ��¼�������ӵ�ʵ������
    // ��������ʽƥ���Ŀ�ĵظ�ʽ
    EntityData goals[MAX_GOALS];
    uint8_t goal_count;
    // ������ida*�� O(1)��ѯĿ�ĵ���Ϣ
    uint8_t goal_type_map[MAP_SIZE]; // ��ʼ��255��ʾ��Ŀ��
    int8_t goal_mask_map[MAP_SIZE];  // ��¼ĳ�������Ӧλͼ�ڼ�λ��ӳ�� û��Ŀ��� -1

    bool cache_valid;
    uint8_t cached_walls[MAP_SIZE];  // �����ж��Ƿ������cached_dist_table
    uint8_t initial_walls[MAP_SIZE]; // ��¼��ʼǽ��ֲ�
    uint16_t cached_dist_table[MAX_GOALS][MAP_SIZE];

    // ����ͳ��
    uint32_t total_explored_nodes;

    // Ȩ������
    float current_weight;
    float min_weight;
    // �������
    MacroAction solution_actions[MAX_STEPS];
    uint8_t solution_actions_len;
} SokobanContext; // 9.54 KB

// ��������ڣ�ִ�� IDA* ����
// ����ֵ��true ��ʾ�ҵ����Ž⣬false ��ʾ�޽��ʱ/���ڵ�����
bool solve(SokobanContext *ctx);

// �ɺ궯�����ݹ켣
// ���� 1: ctx - ����������
// ���� 2: out_full_path - �ⲿ����õĹ켣���飬����Ӧ����Ϊ MAX_STEPS
void generate_path(SokobanContext *ctx, WaypointPath *out_full_path);
// cls :0=无分类  1=分类 2=未知
void build_map_info(SokobanContext *ctx, const uint8_t *raw_map, uint8_t cls);
extern int angle;
#endif // SOKOBAN_ENGINE_H