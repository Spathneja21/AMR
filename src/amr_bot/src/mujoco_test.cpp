#include <mujoco/mujoco.h>
#include <cstdio>

int main() {
    const char* path = "/ros2_ws/src/amr_bot/models/world/world.xml";

    char error[1000] = "";
    mjModel* m = mj_loadXML(path, nullptr, error, 1000);

    if (!m) {
        fprintf(stderr, "mj_loadXML failed: %s\n", error);
        return 1;
    }

    mjData* d = mj_makeData(m);

    for (int i = 0; i < 1000; i++) {
        mj_step(m, d);
    }

    printf("qpos after 1000 steps: x=%f y=%f z=%f\n", d->qpos[0], d->qpos[1], d->qpos[2]);

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}
