#FILE(GLOB task_srcs tasks/*.c)

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    #EMBED_FILES
)
