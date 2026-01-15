#  Pathfinding Algorithms Benchmark (C++)

![C++ Version](https://img.shields.io/badge/C%2B%2B-17%2F20-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)

Kompleksowe narzędzie do analizy i porównywania wydajności najpopularniejszych algorytmów wyszukiwania najkrótszej ścieżki w grafach. Projekt pozwala na testowanie algorytmów na różnych strukturach danych — od prostych grafów nieważonych po grafy z koordynatami geograficznymi.

---

##  Kluczowe Funkcje

* **Trzy potężne algorytmy:**
    * **BFS** – Optymalny dla grafów bez wag (znajduje ścieżkę z najmniejszą liczbą krawędzi).
    * **Dijkstra** – Gwarantuje najkrótszą ścieżkę w grafach z nieujemnymi wagami.
    * **A* (A-Star)** – Inteligentne wyszukiwanie z wykorzystaniem **heurystyki Manhattan** dla grafów z koordynatami XY.
* **Tryb Benchmark (`--compare`):** Automatyczne porównanie czasu wykonania i liczby odwiedzonych węzłów dla wszystkich algorytmów na tym samym grafie. 
* **System Testowy:** Wbudowany moduł `test_all` do weryfikacji poprawności implementacji z oczekiwanymi wynikami. 
* **Obsługa współrzędnych:** Specjalny format `WEIGHTED_XY` pozwalający na symulację map i nawigacji.

---

##  Instalacja i Kompilacja

Projekt nie wymaga zewnętrznych bibliotek — czysty standard C++.


##  Formaty Plików Wejściowych

Program rozpoznaje typ grafu na podstawie nagłówka w pierwszej linii pliku. Każdy format wymaga podania podstawowych parametrów: $n$ (liczba wierzchołków), $m$ (liczba krawędzi), $s$ (start) oraz $t$ (cel).

### 1. UNWEIGHTED (Nieważony)
Przeznaczony głównie dla algorytmu BFS.
```text
UNWEIGHTED
n m
s t
u v
... (m linii)
```
### 2. WEIGHTED (Ważony)
Przeznaczony dla algorytmu Dijkstry.
```text
WEIGHTED
n m
s t
u v w
... (m linii)
```

### 3. WEIGHTED_XY (Ważony z koordynatami)
Wymagany dla algorytmu A*, aby umożliwić obliczenie heurystyki Manhattan.
```text
WEIGHTED_XY
n m
s t
x y       # Koordynaty węzła 0
x y       # Koordynaty węzła 1 (łącznie n linii z XY)
...
u v w     # Krawędzie z wagami (łącznie m linii)
```
---
##  Parametry Linii Komend

Program można uruchomić na dwa sposoby: interaktywnie lub za pomocą argumentów wiersza poleceń.

###  Tryb flag (CLI)
Możesz sterować programem bezpośrednio z konsoli, używając następujących flag:

| Flaga | Opis |
| :--- | :--- |
| `--algo <nazwa>` | Wybór algorytmu: `bfs`, `dijkstra`, `astar` lub `compare`. |
| `--input <ścieżka>` | Ścieżka do pliku tekstowego z danymi grafu. |
| `--compare` | Flaga wymuszająca tryb porównawczy (benchmark) dla wszystkich algorytmów. |

**Przykłady:**
```bash
# Uruchomienie konkretnego algorytmu
./pathfinder --algo dijkstra --input data/graph.txt
```
---
##  Metryki i Wyniki

Po każdym uruchomieniu algorytmu program generuje raport zawierający następujące dane:

* **Status:** Informacja o odnalezieniu ścieżki (`PATH FOUND`) lub jej braku (`NO PATH`).
* **Cost:** Całkowity koszt (suma wag) wyznaczonej trasy.
* **Path:** Pełna sekwencja węzłów tworzących ścieżkę wraz z informacją o całkowitej liczbie węzłów w tej ścieżce.
* **Visited:** Liczba węzłów, które zostały ściągnięte z kolejki w trakcie działania algorytmu (kluczowy wskaźnik wydajności).
* **Time (ms):** Czas trwania obliczeń wyrażony w milisekundach, mierzony przy użyciu precyzyjnego zegara `steady_clock`.

---

##  Struktura Testów

Wbudowany moduł `test_all` pozwala na automatyczną weryfikację poprawności działania algorytmów poprzez porównanie ich wyników z plikami oczekiwanymi:

* **Katalogi testowe:** System wymaga istnienia folderu nadrzędnego `../tests/`, w którym znajdują się podkatalogi `inputs/` oraz `expected/`.
* **Pliki wejściowe:** Dane grafów muszą znajdować się w lokalizacji `../tests/inputs/test[1-5].txt`.
* **Pliki wzorcowe:** Oczekiwane wyniki znajdują się w `../tests/expected/test[1-5].out`.
* **Format pliku `.out`:**
    * Linia zaczynająca się od `COST=` określa spodziewany koszt ścieżki.
    * Linia zaczynająca się od `PATH=` określa spodziewaną sekwencję węzłów oddzielonych spacjami.
* **Logika porównania:**
    * Dla **BFS** system sprawdza jedynie zgodność długości ścieżki (liczby węzłów).
    * Dla **Dijkstry i A*** weryfikowana jest pełna zgodność kosztu oraz całej sekwencji węzłów w ścieżce.
* **Automatyczne pomijanie:** Jeśli dla testu wybrano algorytm BFS, a graf w pliku wejściowym jest oznaczony jako `WEIGHTED`, program automatycznie pominie ten test wyświetlając komunikat `SKIPPED`.
---
##  Architektura Kodu

Program został zaprojektowany w sposób modułowy, co pozwala na łatwą rozbudowę o kolejne algorytmy lub formaty danych.

### Główne Struktury Danych:
* **`Graph`**: Przechowuje reprezentację grafu w formie listy sąsiedztwa (`adj`), informacje o typie grafu (ważony/nieważony) oraz opcjonalne współrzędne wierzchołków (`coords`).
* **`Result`**: Ujednolicony format danych wyjściowych dla wszystkich algorytmów, przechowujący koszt, ścieżkę, liczbę odwiedzonych węzłów oraz czas wykonania.
* **`Expected`**: Struktura pomocnicza używana przez system testowy do wczytywania wzorcowych wyników z plików `.out`.

### Moduły Algorytmiczne:
* **BFS (`bfs_shortest`)**: Wykorzystuje kolejkę `std::queue` oraz automatyczne sortowanie sąsiadów dla zachowania determinizmu wyników.
* **Dijkstra (`dijkstra_shortest`)**: Implementacja oparta na kolejce priorytetowej `std::priority_queue` (min-priority), optymalizująca wyszukiwanie w grafach ważonych.
* **A* (`astar`)**: Rozszerzenie algorytmu Dijkstry o funkcję heurystyczną. Oblicza dystans Manhattan: $h(u) = |x_u - x_t| + |y_u - y_t|$.

### Funkcje Pomocnicze:
* **`read_graph`**: Parser obsługujący trzy różne nagłówki plików i dynamicznie budujący strukturę grafu.
* **`reconstruct`**: Funkcja odtwarzająca sekwencję węzłów na podstawie tablicy "rodziców" wygenerowanej przez algorytmy.
* **System Testowy**: Funkcje `run_test` i `compare_results` automatyzują proces walidacji poprawności kodu.

