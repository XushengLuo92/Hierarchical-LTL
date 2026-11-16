window.DEMOS = [
  {
    "id": 1,
    "title": "Furniture assistance and delivery",
    "description": "A Type 1 robot waits at furniture for Type 3 assistance, then collects outdoor and pet items. A Type 2 robot collects health items before groceries. Each participating robot delivers to packing and returns to the dock.",
    "obligations": [
      "Type 1 stays at furniture until Type 3 arrives.",
      "Type 2 avoids grocery until health is visited.",
      "The same assigned robots carry each order through its delivery tasks."
    ],
    "robots": 6,
    "team": "6 robots",
    "label": "Scenario 1",
    "video": "demo/generated/scenario_01.mp4",
    "poster": "demo/generated/scenario_01.png",
    "paper": "https://arxiv.org/html/2308.10393v3#S2.SS2"
  },
  {
    "id": 2,
    "title": "Three ordered shopping stages",
    "description": "One Type 1 robot completes three shopping stages: furniture and outdoor, then health and grocery, then electronics and pet supplies. It delivers the order to packing and returns to the dock.",
    "obligations": [
      "Finish each shopping stage before starting the next.",
      "Within a stage, either section may be visited first."
    ],
    "robots": 6,
    "team": "6 robots",
    "label": "Scenario 2",
    "video": "demo/generated/scenario_02.mp4",
    "poster": "demo/generated/scenario_02.png",
    "paper": "https://arxiv.org/html/2308.10393v3#S4.SS1"
  },
  {
    "id": 3,
    "title": "Flexible outdoor collection",
    "description": "A Type 1 robot collects health, grocery, electronics, and pet items. The planner chooses either Type 2 or Type 3 for outdoor collection. Both assigned robots deliver their orders and return to the dock.",
    "obligations": [
      "Visit all four Type 1 sections in any order.",
      "Only one outdoor-collection branch is required."
    ],
    "robots": 6,
    "team": "6 robots",
    "label": "Scenario 3",
    "video": "demo/generated/scenario_03.mp4",
    "poster": "demo/generated/scenario_03.png",
    "paper": "https://arxiv.org/html/2308.10393v3#S4.SS1"
  },
  {
    "id": 5,
    "title": "Build the house in layers",
    "description": "Two arms assemble the house through six successive layers. Within each layer, the planner respects the specified placement order and keeps identity-linked brick groups on the same arm.",
    "obligations": [
      "Complete each layer before starting the next.",
      "In layer 5, place p15 and p16 before p17."
    ],
    "robots": 2,
    "team": "2 arms \u00b7 schematic plan",
    "label": "Scenario 4",
    "video": "demo/generated/scenario_05.mp4",
    "poster": "demo/generated/scenario_05.png",
    "paper": "https://arxiv.org/html/2308.10393v3#S4.SS2.SSS2"
  }
];
