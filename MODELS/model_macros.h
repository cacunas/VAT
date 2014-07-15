#ifndef MODEL_MACROS_H
#define MODEL_MACROS_H

#define ADD_MODEL(model) \
ReliabilitySingleModelInterface *ALLOCATE_##model () { \
    ReliabilitySingleModelInterface *m = new model(); \
    m->model_name = #model; \
    return m; }


#define ALLOCATE_MODEL(model) ALLOCATE_##model()

#define REGISTER_MODEL(name) \
VideoAnalysis::availableModels.insert(#name); \
VideoAnalysis::modelConstructor[#name] = &ALLOCATE_##name;

#endif // MODULE_MACROS_H
