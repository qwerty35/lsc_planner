\ENCODING=ISO-8859-1
\Problem name: IloCplex_conflict

Minimize
 obj1:
Subject To
 c1:   x_0_0  = -0.409839987754822
 c2:   - 25 x_0_0 + 25 x_0_1  = 0
 c3:   500 x_0_0 - 1000 x_0_1 + 500 x_0_2  = 0
 c16:  y_0_0  = -1.39649999141693
 c17:  - 25 y_0_0 + 25 y_0_1  = 0
 c18:  500 y_0_0 - 1000 y_0_1 + 500 y_0_2  = 0
 c127: 0.127187848091125 y_0_3 - 0.991878628730774 x_0_3 >= 0.271965308606915
 c296: - 500 x_0_3 - 500 x_0_1 + 1000 x_0_2 <= 2
 c379: 500 y_0_3 + 500 y_0_1 - 1000 y_0_2 <= 2
\Sum of equality rows in the conflict:
\ sum_eq: 476 y_0_0 - 975 y_0_1 + 500 y_0_2 + 476 x_0_0 - 975 x_0_1 + 500 x_0_2
\          = -1.80633997917175
Bounds
      y_0_3 Free
      y_0_0 Free
      y_0_1 Free
      y_0_2 Free
      x_0_3 Free
      x_0_0 Free
      x_0_1 Free
      x_0_2 Free
End
