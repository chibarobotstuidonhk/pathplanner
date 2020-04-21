#pragma once

#include <GLFW/glfw3.h>
#include <math.h>

// 1px = 1cmとして描画
// psoptの出力はmなので注意

void drawSquare(const GLfloat vtx[], const GLfloat color[])
{
    glVertexPointer(2, GL_FLOAT, 0, vtx);
    glColor4f(color[0], color[1], color[2], color[3]);

    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_QUADS, 0, 4);
    glDisableClientState(GL_VERTEX_ARRAY);
}
void drawCircle(GLfloat x, GLfloat y, GLfloat radius, const GLfloat color[])
{
    GLfloat vtx[32 * 2];
    for (int i = 0; i < 32; ++i)
    {
        GLfloat angle = static_cast<GLfloat>((M_PI * 2.0 * i) / 32);
        vtx[i * 2] = radius * sin(angle) + x;
        vtx[i * 2 + 1] = radius * cos(angle) + y;
    }

    glVertexPointer(2, GL_FLOAT, 0, vtx);
    glColor4f(color[0], color[1], color[2], color[3]);

    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 32);
    glDisableClientState(GL_VERTEX_ARRAY);
}
static const GLfloat brown[] = {
    0.57f,
    0.44f,
    0.35f,
    1.0f,
};
static const GLfloat green[] = {
    0.38f,
    0.62f,
    0.28f,
    1.0f,
};
static const GLfloat red[] = {
    0.69f,
    0.19f,
    0.21f,
    1.0f,
};
static const GLfloat lightred[] = {
    0.91f,
    0.43f,
    0.42f,
    1.0f,
};
static const GLfloat mediumred[] = {
    0.75f,
    0.30f,
    0.24f,
    1.0f,
};
static const GLfloat blue[] = {
    0.00f,
    0.42f,
    0.67f,
    1.0f,
};
static const GLfloat lightblue[] = {
    0.53f,
    0.79f,
    0.91f,
    1.0f,
};
static const GLfloat mediumblue[] = {
    0.30f,
    0.68f,
    0.87f,
    1.0f,
};
static const GLfloat lightgreen[] = {
    0.49f,
    0.76f,
    0.31f,
    1.0f,
};
static const GLfloat yellow[] = {
    0.99f,
    0.90f,
    0.60f,
    1.0f,
};
static const GLfloat fense1_r[] = {
    0.0f,
    0.0f,
    0.0f,
    5.0f,
    670.0f,
    5.0f,
    670.0f,
    0.0f,
};
static const GLfloat fense2_r[] = {
    0.0f,
    0.0f,
    5.0f,
    0.0f,
    5.0f,
    1010.0f,
    0.0f,
    1010.0f,
};
static const GLfloat fense3_r[] = {
    0.0f,
    1010.0f,
    670.0f,
    1010.0f,
    670.0f,
    1005.0f,
    0.0f,
    1005.0f,
};
static const GLfloat fense4_r[] = {
    410.0f,
    5.0f,
    415.0f,
    5.0f,
    415.0f,
    314.0f,
    410.0f,
    314.0f,
};
static const GLfloat fense5_r[] = {
    410.0f,
    1005.0f,
    415.0f,
    1005.0f,
    415.0f,
    846.0f,
    410.0f,
    846.0f,
};
static const GLfloat rz_r[] = {
    5.0f,
    5.0f,
    162.5f,
    5.0f,
    162.5f,
    1005.0f,
    5.0f,
    1005.0f,
};
static const GLfloat kz1_r[] = {
    162.5f,
    505.0f,
    412.5f,
    505.0f,
    412.5f,
    1005.0f,
    162.5f,
    1005.0f,
};
static const GLfloat kz2_r[] = {
    162.5f,
    255.0f,
    412.5f,
    255.0f,
    412.5f,
    505.0f,
    162.5f,
    505.0f,
};
static const GLfloat kz3_r[] = {
    162.5f,
    5.0f,
    412.5f,
    5.0f,
    412.5f,
    255.0f,
    162.5f,
    255.0f,
};
static const GLfloat pz_r[] = {
    412.5f,
    5.0f,
    620.0f,
    5.0f,
    620.0f,
    1005.0f,
    412.5f,
    1005.0f,
};
static const GLfloat bz_r[] = {
    620.0f,
    5.0f,
    670.0f,
    5.0f,
    670.0f,
    1005.0f,
    620.0f,
    1005.0f,
};
static const GLfloat ts1_r[] = {
    621.0f,
    285.0f,
    670.0f,
    285.0f,
    670.0f,
    343.0f,
    621.0f,
    343.0f,
};
static const GLfloat ts2_r[] = {
    621.0f,
    418.0f,
    670.0f,
    418.0f,
    670.0f,
    476.0f,
    621.0f,
    476.0f,
};
static const GLfloat ts3_r[] = {
    621.0f,
    551.0f,
    670.0f,
    551.0f,
    670.0f,
    609.0f,
    621.0f,
    609.0f,
};
static const GLfloat ts4_r[] = {
    621.0f,
    684.0f,
    670.0f,
    684.0f,
    670.0f,
    742.0f,
    621.0f,
    742.0f,
};
static const GLfloat ts5_r[] = {
    621.0f,
    817.0f,
    670.0f,
    817.0f,
    670.0f,
    875.0f,
    621.0f,
    875.0f,
};
static const GLfloat trsz_r[] = {
    5.0f,
    1005.0f,
    105.0f,
    1005.0f,
    105.0f,
    905.0f,
    5.0f,
    905.0f,
};
static const GLfloat prsz_r[] = {
    413.0f,
    5.0f,
    513.0f,
    5.0f,
    513.0f,
    105.0f,
    413.0f,
    105.0f,
};
static const GLfloat fense1_b[] = {
    0.0f,
    0.0f,
    0.0f,
    5.0f,
    670.0f,
    5.0f,
    670.0f,
    0.0f,
};
static const GLfloat fense2_b[] = {
    665.0f,
    0.0f,
    670.0f,
    0.0f,
    670.0f,
    1010.0f,
    665.0f,
    1010.0f,
};
static const GLfloat fense3_b[] = {
    0.0f,
    1010.0f,
    670.0f,
    1010.0f,
    670.0f,
    1005.0f,
    0.0f,
    1005.0f,
};
static const GLfloat fense4_b[] = {
    255.0f,
    5.0f,
    260.0f,
    5.0f,
    260.0f,
    314.0f,
    255.0f,
    314.0f,
};
static const GLfloat fense5_b[] = {
    255.0f,
    1005.0f,
    260.0f,
    1005.0f,
    260.0f,
    846.0f,
    255.0f,
    846.0f,
};
static const GLfloat rz_b[] = {
    507.5f,
    5.0f,
    665.0f,
    5.0f,
    665.0f,
    1005.0f,
    507.5f,
    1005.0f,
};
static const GLfloat kz1_b[] = {
    257.5f,
    505.0f,
    507.5f,
    505.0f,
    507.5f,
    1005.0f,
    257.5f,
    1005.0f,
};
static const GLfloat kz2_b[] = {
    257.5f,
    255.0f,
    507.5f,
    255.0f,
    507.5f,
    505.0f,
    257.5f,
    505.0f,
};
static const GLfloat kz3_b[] = {
    257.5f,
    5.0f,
    507.5f,
    5.0f,
    507.5f,
    255.0f,
    257.5f,
    255.0f,
};
static const GLfloat pz_b[] = {
    50.0f,
    5.0f,
    257.5f,
    5.0f,
    257.5f,
    1005.0f,
    50.0f,
    1005.0f,
};
static const GLfloat bz_b[] = {
    0.0f,
    5.0f,
    50.0f,
    5.0f,
    50.0f,
    1005.0f,
    0.0f,
    1005.0f,
};
static const GLfloat ts1_b[] = {
    0.0f,
    285.0f,
    49.0f,
    285.0f,
    49.0f,
    343.0f,
    0.0f,
    343.0f,
};
static const GLfloat ts2_b[] = {
    0.0f,
    418.0f,
    49.0f,
    418.0f,
    49.0f,
    476.0f,
    0.0f,
    476.0f,
};
static const GLfloat ts3_b[] = {
    0.0f,
    551.0f,
    49.0f,
    551.0f,
    49.0f,
    609.0f,
    0.0f,
    609.0f,
};
static const GLfloat ts4_b[] = {
    0.0f,
    684.0f,
    49.0f,
    684.0f,
    49.0f,
    742.0f,
    0.0f,
    742.0f,
};
static const GLfloat ts5_b[] = {
    0.0f,
    817.0f,
    49.0f,
    817.0f,
    49.0f,
    875.0f,
    0.0f,
    875.0f,
};
static const GLfloat trsz_b[] = {
    565.0f,
    1005.0f,
    665.0f,
    1005.0f,
    665.0f,
    905.0f,
    565.0f,
    905.0f,
};
static const GLfloat prsz_b[] = {
    155.0f,
    5.0f,
    255.0f,
    5.0f,
    255.0f,
    105.0f,
    155.0f,
    105.0f,
};

void drawRedField()
{
    drawSquare(rz_r, green);
    drawSquare(kz1_r, lightred);
    drawSquare(kz2_r, mediumred);
    drawSquare(kz3_r, red);
    drawSquare(pz_r, lightgreen);
    drawSquare(bz_r, yellow);
    drawSquare(ts1_r, green);
    drawSquare(ts2_r, green);
    drawSquare(ts3_r, green);
    drawSquare(ts4_r, green);
    drawSquare(ts5_r, green);
    drawSquare(prsz_r, red);
    drawSquare(trsz_r, red);
    drawSquare(fense1_r, brown);
    drawSquare(fense2_r, brown);
    drawSquare(fense3_r, brown);
    drawSquare(fense4_r, brown);
    drawSquare(fense5_r, brown);
    drawCircle(412.5f, 314.0f, 10.7f, blue);
    drawCircle(412.5f, 580.0f, 10.7f, blue);
    drawCircle(412.5f, 846.0f, 10.7f, blue);
    drawCircle(261.0f, 713.0f, 10.7f, blue);
    drawCircle(261.0f, 447.0f, 10.7f, blue);
    drawCircle(309.2f, 246.9f, 8.5f, yellow);
}

void drawBlueField()
{
    drawSquare(rz_b, green);
    drawSquare(kz1_b, lightblue);
    drawSquare(kz2_b, mediumblue);
    drawSquare(kz3_b, blue);
    drawSquare(pz_b, lightgreen);
    drawSquare(bz_b, yellow);
    drawSquare(ts1_b, green);
    drawSquare(ts2_b, green);
    drawSquare(ts3_b, green);
    drawSquare(ts4_b, green);
    drawSquare(ts5_b, green);
    drawSquare(prsz_b, blue);
    drawSquare(trsz_b, blue);
    drawSquare(fense1_b, brown);
    drawSquare(fense2_b, brown);
    drawSquare(fense3_b, brown);
    drawSquare(fense4_b, brown);
    drawSquare(fense5_b, brown);
    drawCircle(257.5f, 314.0f, 10.7f, red);
    drawCircle(257.5f, 580.0f, 10.7f, red);
    drawCircle(257.5f, 846.0f, 10.7f, red);
    drawCircle(409.0f, 713.0f, 10.7f, red);
    drawCircle(409.0f, 447.0f, 10.7f, red);
    drawCircle(317.0f, 218.0f, 8.5f, yellow);
}