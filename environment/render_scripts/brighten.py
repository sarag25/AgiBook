def brighten(acts, flat_black=False):
    """flat_black (portapenne): la texture e' nero pieno e in PBR diventa una sagoma
    piatta -> tolgo la texture, grafite scuro in Phong con riflesso per leggerne la forma.
    Oggetti con texture colorata (mappamondo): restano PBR."""
    if not flat_black: return acts
    for a in acts:
        p=a.GetProperty();p.RemoveAllTextures();a.GetMapper().ScalarVisibilityOff()
        p.SetInterpolationToPhong();p.SetColor(0.12,0.12,0.13)
        p.SetAmbient(0.3);p.SetDiffuse(0.8);p.SetSpecular(0.7);p.SetSpecularPower(40);p.SetSpecularColor(1,1,1)
    return acts
