---
layout: page
title: Publications
show_sidebar: false
hide_footer: false
---

<style>
.csl-block {
    font-size: 16px;
}
.csl-title, .csl-author, .csl-event, .csl-editor, .csl-venue {
    display: block;
    position: relative;
    font-size: 16px;
}

.csl-title b {
    font-weight: 600;
}

.csl-content {
    display: inline-block;
    vertical-align: top;
    padding-left: 20px;
}

.bibliography {
  list-style-type: none;
}

.bibliography > li::marker {
  content: "[" counter(list-item) "]";
  counter-increment: list;
}

</style>

# 2025
{% bibliography --query @*[year=2025] %}

# 2024
{% bibliography --query @*[year=2024] %}

# 2023
{% bibliography --query @*[year=2023] %}

# 2022
{% bibliography --query @*[year=2022] %}

# 2021
{% bibliography --query @*[year=2021] %}


