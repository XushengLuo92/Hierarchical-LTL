"""Presentation copy aligned with the README examples and their specifications."""
PAPER = 'https://arxiv.org/html/2308.10393v3'
TITLES = ['Furniture assistance and delivery', 'Three ordered shopping stages',
          'Flexible outdoor collection', 'Build the ICL! model', 'Build the house in layers']
DESCRIPTIONS = [
    'A Type 1 robot waits at furniture for Type 3 assistance, then collects outdoor and pet items. A Type 2 robot collects health items before groceries. Each participating robot delivers to packing and returns to the dock.',
    'One Type 1 robot completes three shopping stages: furniture and outdoor, then health and grocery, then electronics and pet supplies. It delivers the order to packing and returns to the dock.',
    'A Type 1 robot collects health, grocery, electronics, and pet items. The planner chooses either Type 2 or Type 3 for outdoor collection. Both assigned robots deliver their orders and return to the dock.',
    'Two arms share the brick-placement tasks for I, C, and L. Each letter follows its own three-step sequence. The exclamation mark is assembled after all three letters are complete.',
    'Two arms assemble the house through six successive layers. Within each layer, the planner respects the specified placement order and keeps identity-linked brick groups on the same arm.'
]
REQUIREMENTS = [
    ['Type 1 stays at furniture until Type 3 arrives.', 'Type 2 avoids grocery until health is visited.', 'The same assigned robots carry each order through its delivery tasks.'],
    ['Finish each shopping stage before starting the next.', 'Within a stage, either section may be visited first.'],
    ['Visit all four Type 1 sections in any order.', 'Only one outdoor-collection branch is required.'],
    ['Complete I, C, and L before the exclamation mark.', 'This is the README’s ICL! example from the earlier paper version; the published paper uses bin packing instead.'],
    ['Complete each layer before starting the next.', 'In layer 5, place p15 and p16 before p17.']
]
CARDS = {
 ('nav',5): [('p201','Furniture handoff','F(furniture₁ ∧ X(furniture₁ U furniture₃))'),
             ('p202','Outdoor + pet supplies','F outdoor₁ ∧ F pet₁'),
             ('p203','Type 1 delivery','F(packing₁ ∧ F dock₁)'),
             ('p204','Health before grocery','F health₂ ∧ F grocery₂ ∧ (¬grocery₂ U health₂)'),
             ('p205','Type 2 delivery','F(packing₂ ∧ F dock₂)'),
             ('p206','Type 3 delivery','F(packing₃ ∧ F dock₃)')],
 ('nav',6): [('p101','Stage 1 · furniture + outdoor','F furniture₁ ∧ F outdoor₁'),
             ('p102','Stage 2 · health + grocery','F health₁ ∧ F grocery₁'),
             ('p103','Stage 3 · electronics + pet','F electronics₁ ∧ F pet₁'),
             ('p104','Deliver + return','F(packing₁ ∧ F dock₁)')],
 ('nav',8): [('p101','Collect four sections','F health₁ ∧ F grocery₁ ∧ F electronics₁ ∧ F pet₁'),
             ('p102','Type 1 delivery','F(packing₁ ∧ F dock₁)'),
             ('p103','Outdoor · Type 2 option','F(outdoor₂ ∧ F(packing₂ ∧ F dock₂))'),
             ('p104','Outdoor · Type 3 option','F(outdoor₃ ∧ F(packing₃ ∧ F dock₃))')],
 ('man',5): [('p300','Build I','F(p3 ∧ F(p4 ∧ F p5))'),
             ('p400','Build C','F(p6 ∧ F(p7 ∧ F p8))'),
             ('p500','Build L','F(p9 ∧ F(p10 ∧ F p11))'),
             ('p200','Add !','F p1 ∧ F p2; after I, C, and L')],
 ('man',6): [('p100','Layer 1 · foundation','F p101 ∧ F p102'),
             ('p200','Layer 2','F p201 ∧ F p202'),
             ('p300','Layer 3','F p301 ∧ F p302'),
             ('p400','Layer 4','F p401'),
             ('p500','Layer 5','F(p501 ∧ F p502)'),
             ('p600','Layer 6 · finish','F p601')]
}
REGIONS = dict(zip([f'p{i}' for i in range(8)],
                   ['Dock','Grocery','Health','Outdoor','Pet supplies','Furniture','Electronics','Packing']))
