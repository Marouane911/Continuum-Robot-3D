class CTRConstraints:

    @staticmethod
    def enforce_telescopic_constraints(q, active, entrainement=True,tubes_lengths=None):

        if tubes_lengths is None:
            l1, l2, l3 = 0.463, 0.3305, 0.199
        else:
            l1, l2, l3 = tubes_lengths[0], tubes_lengths[1], tubes_lengths[2]

        eps = 0.005
        
        max_beta3 = 0 # Je sais pas si c'est correct
        min_beta3 = -l3+0.01 # Je sais pas si c'est correct

        q = list(q)

        # SÉCURITÉ : Limites physiques (On empêche de sortir des rails)
        q[0] = max(-l1, min(0.0, q[0]))
        q[1] = max(-l2, min(0.0, q[1]))
        q[2] = max(-l3, min(0.0, q[2]))

        # MODE 1 : ENTRAÎNEMENT (Le chariot actif pousse/tire les autres)
        if entrainement:
            
            # 1. Murs physiques (q[2] entraîne tout le monde vers l'avant/arrière)
            if q[2] > max_beta3:
                q[2] = max_beta3
                if q[1] > q[2] - eps: q[1] = q[2] - eps
                if q[0] > q[1] - eps: q[0] = q[1] - eps

            if q[2] < min_beta3:
                q[2] = min_beta3
                tip3, tip2 = l3 + q[2], l2 + q[1]
                if tip3 > tip2 - eps:
                    q[1] = q[2] - (l2 - l3) + eps
                    tip2 = l2 + q[1]
                    tip1 = l1 + q[0]
                    if tip2 > tip1 - eps:
                        q[0] = q[1] - (l1 - l2) + eps

            # 2. Collision des bases
            if active == 0:
                if q[1] < q[0] + eps: q[1] = q[0] + eps
                if q[2] < q[1] + eps: q[2] = q[1] + eps
            elif active == 2:
                if q[1] > q[2] - eps: q[1] = q[2] - eps
                if q[0] > q[1] - eps: q[0] = q[1] - eps
            elif active == 1:
                if q[2] < q[1] + eps: q[2] = q[1] + eps
                if q[0] > q[1] - eps: q[0] = q[1] - eps

            # 3. Sécurité des pointes
            tip1, tip2, tip3 = l1 + q[0], l2 + q[1], l3 + q[2]
            
            if active == 0:
                if tip2 > tip1 - eps:
                    q[1] = q[0] + (l1 - l2) - eps
                    tip2 = l2 + q[1] 
                if tip3 > tip2 - eps:
                    q[2] = q[1] + (l2 - l3) - eps
            elif active == 2:
                if tip3 > tip2 - eps:
                    q[1] = q[2] - (l2 - l3) + eps
                    tip2 = l2 + q[1]
                if tip2 > tip1 - eps:
                    q[0] = q[1] - (l1 - l2) + eps
            elif active == 1:
                if tip2 > tip1 - eps: q[0] = q[1] - (l1 - l2) + eps
                if tip3 > tip2 - eps: q[2] = q[1] + (l2 - l3) - eps
                
            # 4. Double check final vers l'arrière
            if q[1] > q[2] - eps: q[1] = q[2] - eps
            if q[0] > q[1] - eps: q[0] = q[1] - eps


        # MODE 2 : BLOCAGE (Le chariot actif s'arrête s'il tape un obstacle)
        else:
            # 1. Murs physiques stricts pour q[2]
            if q[2] > max_beta3: q[2] = max_beta3
            if q[2] < min_beta3: q[2] = min_beta3
            
            # 2. Collision des bases (Le chariot actif est stoppé par les passifs)
            if active == 0:
                # Tube 0 avance : il est bloqué par Tube 1
                if q[0] > q[1] - eps: q[0] = q[1] - eps
            elif active == 2:
                # Tube 2 recule : il est bloqué par Tube 1
                if q[2] < q[1] + eps: q[2] = q[1] + eps
            elif active == 1:
                # Tube 1 peut être bloqué en avançant (par 2) ou en reculant (par 0)
                if q[1] > q[2] - eps: q[1] = q[2] - eps
                if q[1] < q[0] + eps: q[1] = q[0] + eps

            # 3. Sécurité des pointes (Le chariot actif est stoppé s'il "avale" un autre)
            tip1, tip2, tip3 = l1 + q[0], l2 + q[1], l3 + q[2]
            
            if active == 0:
                # Tube 0 recule : sa pointe ne peut pas rentrer dans le tube 1
                if tip1 < tip2 + eps: 
                    q[0] = tip2 + eps - l1
            elif active == 2:
                # Tube 2 avance : sa pointe ne peut pas avaler le tube 1
                if tip3 > tip2 - eps: 
                    q[2] = tip2 - eps - l3
            elif active == 1:
                # Tube 1 avance : sa pointe ne peut pas avaler le tube 0
                if tip2 > tip1 - eps: 
                    q[1] = tip1 - eps - l2
                # Tube 1 recule : sa pointe ne peut pas rentrer dans le tube 2
                if tip2 < tip3 + eps: 
                    q[1] = tip3 + eps - l2

            # 4. Double check final classique
            if q[1] < q[0] + eps: q[1] = q[0] + eps
            if q[2] < q[1] + eps: q[2] = q[1] + eps

        return q