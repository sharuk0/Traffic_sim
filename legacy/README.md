# Archivos heredados

## `test_cases_legacy.csv`

CSV del proyecto original. **Su procedencia no pudo verificarse** y no se le atribuye
ninguna: no existe código en el proyecto original que lo lea ni lo escriba, sus coordenadas
(`start_x` hasta 1982) son incompatibles con el mundo simulado (`[-312, 312]²`), su destino
es siempre `(1000, 0)` —que no es un nodo de la intersección—, y su columna `traffic_density`
es continua en `[51, 297]` cuando el código sólo podía producir 100 o 200.

La correlación entre `traffic_density` y `time_to_reach_destination` es r = 0.752, coherente
con datos sintéticos generados por una fórmula con ruido.

Se conserva sólo como referencia histórica. El mecanismo reproducible que lo reemplaza es
`scripts/generate_test_cases.py`.
