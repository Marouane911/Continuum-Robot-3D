class CTRConstraints:

    @staticmethod
    def enforce_telescopic_constraints(q, active):
        l1 = 0.463
        l2 = 0.3305
        l3 = 0.199

        eps = 0.005
        q = list(q)

        # Phase 1 : Sécurité des chariots à la base (Proximal)
        if q[1] < q[0] + eps:
            q[1] = q[0] + eps
        if q[2] < q[1] + eps:
            q[2] = q[1] + eps

        # Phase 2 : Sécurité des pointes des tubes (Distal)
        tip1 = l1 + q[0]
        tip2 = l2 + q[1]
        tip3 = l3 + q[2]

        if tip2 > tip1 - eps:
            q[1] = q[0] + (l1 - l2) - eps
            tip2 = l2 + q[1] 

        if tip3 > tip2 - eps:
            q[2] = q[1] + (l2 - l3) - eps

        # Phase 3 : Double-check de sécurité pour figer les bases
        if q[1] < q[0] + eps: q[1] = q[0] + eps
        if q[2] < q[1] + eps: q[2] = q[1] + eps

        # ---------------------------------------------------------
        # Phase 4 : Limite de l'Espace de Travail Sûr (Anti-Divergence)
        # ---------------------------------------------------------
        max_beta3 = -0.005 # La limite empirique avant le crash du solver
        
        if q[2] > max_beta3:
            q[2] = max_beta3
            
            # Si on bloque q[2], on repousse q[1] et q[0] vers l'arrière 
            # pour maintenir l'écart de sécurité 'eps' et ne pas écraser les chariots
            if q[1] > q[2] - eps: 
                q[1] = q[2] - eps
            if q[0] > q[1] - eps: 
                q[0] = q[1] - eps

        return q