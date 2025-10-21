---
title: LIAS
subtitle: Welcome to The Laboratory for Intelligent Autonomous Systems!
layout: page
show_sidebar: false
hide_footer: false
hero_height: is-large
hero_image: /img/homepage_slideshow.gif
hero_link: /research/
hero_link_text: Our Research

hero_link2: /current-members/
hero_link_text2: Our Team
---

# Who are we?
We are the Laboratory for Intelligent Autonomous Systems from [School of Data Scinece](https://sds.cuhk.edu.cn/) at [The Chinese University of HongKong, ShenZhen](https://www.cuhk.edu.cn/).

Our research focus on ...

# Highlights
{% assign posts = site.posts | where:"categories","highlights" %}
<div class="columns is-multiline">
    {% for post in posts %}
    <div class="column is-4-desktop is-6-tablet">
        {% include post-card.html %}
    </div>
    {% endfor %}
</div>
