class CTRConstraints:

    @staticmethod
    def enforce_telescopic_constraints(q, active):
        # Longueurs physiques fixes de tes tubes
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
            tip2 = l2 + q[1] # Recalcul pour la chaîne suivante

        if tip3 > tip2 - eps:
            q[2] = q[1] + (l2 - l3) - eps

        # Phase 3 : Double-check de sécurité pour figer les bases
        if q[1] < q[0] + eps: q[1] = q[0] + eps
        if q[2] < q[1] + eps: q[2] = q[1] + eps

        return q