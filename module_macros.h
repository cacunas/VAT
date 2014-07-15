#ifndef MODULE_MACROS_H
#define MODULE_MACROS_H

#define ADD_MODULE(module) \
ModuleInterface *ALLOCATE_##module (Datapool *i_data) { \
    ModuleInterface *m = new module(i_data); \
    m->name = #module; \
    m->type = VideoAnalysis::moduleType[#module]; \
    return m; }

#define ALLOCATE_MODULE(module) ALLOCATE_##module()

#define REGISTER_MODULE(name, type) \
VideoAnalysis::availableModules.insert(#name); \
VideoAnalysis::moduleType[#name] = type; \
VideoAnalysis::moduleConstructor[#name] = &ALLOCATE_##name; \
VideoAnalysis::modulesByType.insert(std::pair<ModuleType, std::string>(type, #name));
#endif // MODULE_MACROS_H
