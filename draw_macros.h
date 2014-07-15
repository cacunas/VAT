#ifndef DRAW_MACROS_H
#define DRAW_MACROS_H

#define ADD_DRAW(draw) \
drawInterface *ALLOCATE_##draw (Datapool *i_data) { \
    drawInterface *m = new draw(i_data); \
    m->name = #draw; \
    return m; }

#define ALLOCATE_DRAW(draw) ALLOCATE_##draw()

/*
#define REGISTER_DRAW(name) \
VideoAnalysis::drawConstructor[#name] = &ALLOCATE_##name; \
*/

#define REGISTER_DRAW_PAINT(name) \
    paintView::drawConstructor[#name] = &ALLOCATE_##name; \

#endif // DRAW_MACROS_H
