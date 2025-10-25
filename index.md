---
title: LIAS
subtitle: Welcome to The Laboratory for Intelligent Autonomous Systems!
layout: page
show_sidebar: false
hide_footer: false
hero_height: is-large
hero_image: /img/homepage_slideshow.gif
hero_link: /publications/
hero_link_text: Academic Paper

hero_link2: /current-members/
hero_link_text2: Team
---

# Who are we?
We are the Laboratory for Intelligent Autonomous Systems from [School of Data Scinece](https://sds.cuhk.edu.cn/) at [The Chinese University of HongKong, ShenZhen](https://www.cuhk.edu.cn/).

Our research focus on:

* Underwater intelligent autonomous robotics
* Localization, state estimation and navigation in robotics
* Systems theory, decision and control theory, and autonomy
* Network systems, from multi-agent systems, sensor networks to social networks
* Safety, security and privacy in information systems, such as cyber physical systems, machine learning algorithms

# News
{% assign posts = site.posts | where:"categories","news" %}
<div class="columns is-multiline">
    {% for post in posts %}
    <div class="column is-4-desktop is-6-tablet">
        {% include post-card.html %}
    </div>
    {% endfor %}
</div>
