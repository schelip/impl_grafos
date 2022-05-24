from enum import Enum


class Color(Enum):
    """
    Classe definindo as possíveis cores de um Vértice, utilizadas nos
    algoritmos para representar seu estado de 'descobrimento'
    """
    WHITE = 1  # Não Descoberto
    GRAY = 2  # Descoberto
    BLACK = 3  # Finalizado
