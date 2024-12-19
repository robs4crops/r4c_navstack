import matplotlib.pyplot as plt

def generar_puntos(A, B, O):
    # Calcular el vector AB
    vector_AB = [B[0] - A[0], B[1] - A[1]]

    # Normalizar el vector AB
    magnitud = (vector_AB[0]**2 + vector_AB[1]**2)**0.5
    vector_normalizado_AB = [vector_AB[0] / magnitud, vector_AB[1] / magnitud]

    # Calcular el desplazamiento
    desplazamiento = [vector_normalizado_AB[0] * O, vector_normalizado_AB[1] * O]

    # Calcular las coordenadas de C y D
    C = [A[0] + desplazamiento[0], A[1] + desplazamiento[1]]
    D = [B[0] - desplazamiento[0], B[1] - desplazamiento[1]]

    return C, D

# Puntos originales
A = [7, 10]
B = [-5, 5]

# Distancia de acercamiento
O = 2

# Generar los puntos C y D
C, D = generar_puntos(A, B, O)

# Graficar los puntos
plt.plot(A[0], A[1], 'bo', label='A')
plt.plot(B[0], B[1], 'bo', label='B')
plt.plot(C[0], C[1], 'ro', label='C')
plt.plot(D[0], D[1], 'ro', label='D')

# Agregar anotaciones
plt.text(A[0], A[1], 'A', verticalalignment='bottom', horizontalalignment='right')
plt.text(B[0], B[1], 'B', verticalalignment='bottom', horizontalalignment='right')
plt.text(C[0], C[1], 'C', verticalalignment='bottom', horizontalalignment='right')
plt.text(D[0], D[1], 'D', verticalalignment='bottom', horizontalalignment='right')

# Graficar el segmento AB
plt.plot([A[0], B[0]], [A[1], B[1]], 'b--', label='AB')

# Establecer límites del gráfico
plt.xlim(min(A[0], B[0], C[0], D[0]) - 1, max(A[0], B[0], C[0], D[0]) + 1)
plt.ylim(min(A[1], B[1], C[1], D[1]) - 1, max(A[1], B[1], C[1], D[1]) + 1)

# Etiquetas de ejes y título
plt.xlabel('Coordenada X')
plt.ylabel('Coordenada Y')
plt.title('Puntos A, B, C y D')

# Leyenda
plt.legend()

# Mostrar la gráfica
plt.grid(True)
plt.show()